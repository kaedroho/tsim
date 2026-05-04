package main

// Odin port of the Rust map generator.
//
// The Rust original uses three crates that don't have ready Odin equivalents:
//   - voronoice     -> Voronoi diagram + Lloyd relaxation
//   - fast_poisson  -> Bridson Poisson-disk sampling
//   - kiddo         -> KD-tree nearest-neighbour
//
// The interfaces for those are stubbed below (search for `// STUB`). Plug in
// real implementations there and the rest of the module runs as-is. Bevy's
// ECS bits are dropped — this is just a library + main().

import "core:fmt"
import "core:math"
import "core:math/noise"
import "core:math/rand"
import vmem "core:mem/virtual"
import "core:os"
import rl "vendor:raylib"

// =============================================================================
// Cell types
// =============================================================================

CellType :: enum u8 {
	Land,
	Water,
	Headland,
	Beach,
	Cove,
	Channel,
	Ocean,
}

cell_is_land :: proc(ct: CellType) -> bool {
	#partial switch ct {
	case .Land, .Headland, .Beach:
		return true
	}
	return false
}

// =============================================================================
// VoronoiMap
// =============================================================================

VoronoiMap :: struct {
	width, height: u32,
	positions:     [dynamic][2]f32,
	heights:       []f32,
	cell_types:    []CellType,
	kdtree:        ^KDTreeNode,
	kdtree_arena:  vmem.Arena,
	voronoi:       Voronoi,
}

InitCellProc :: #type proc(position: [2]f32, user_data: rawptr) -> (CellType, f32)

voronoi_map_make :: proc(
	width, height: u32,
	radius: f32,
	seed: u32,
	init_cell: InitCellProc,
	user_data: rawptr = nil,
) -> VoronoiMap {
	positions := poisson_generate(f32(width), f32(height), radius, u64(seed))

	kdtree_arena: vmem.Arena
	kdtree_arena_allocator := vmem.arena_allocator(&kdtree_arena)
	kdtree := kdtree_build(positions[:], allocator = kdtree_arena_allocator)

	cell_types := make([]CellType, len(positions))
	heights := make([]f32, len(positions))
	for i in 0 ..< len(positions) {
		cell_types[i], heights[i] = init_cell(positions[i], user_data)
	}

	voronoi := voronoi_build(positions[:], f32(width), f32(height), 5)

	return {
		width = width,
		height = height,
		positions = positions,
		heights = heights,
		cell_types = cell_types,
		kdtree = kdtree,
		kdtree_arena = kdtree_arena,
		voronoi = voronoi,
	}
}

voronoi_map_destroy :: proc(m: ^VoronoiMap) {
	vmem.arena_destroy(&m.kdtree_arena)
	voronoi_destroy(&m.voronoi)
	delete(m.positions)
	delete(m.heights)
	delete(m.cell_types)
}

// Returns the two endpoints of the Voronoi edge shared by cells a and b.
get_edge_between_cells :: proc(m: ^VoronoiMap, a, b: int) -> (start, end: [2]f32, ok: bool) {
	first_shared := -1
	for ta in m.voronoi.cells[a].triangles {
		for tb in m.voronoi.cells[b].triangles {
			if ta == tb {
				if first_shared == -1 {
					first_shared = int(ta)
				} else {
					return m.voronoi.vertices[first_shared], m.voronoi.vertices[ta], true
				}
			}
		}
	}
	return {}, {}, false
}

// Connected-component labelling for water cells.
// `cell_to_body[i]` is the waterbody index for cell i, or -1 if it's land.
find_waterbodies :: proc(m: ^VoronoiMap) -> (bodies: [][]int, cell_to_body: []int) {
	n := len(m.positions)
	cell_to_body = make([]int, n)
	for i in 0 ..< n do cell_to_body[i] = -1

	visited := make([]bool, n); defer delete(visited)
	queue: [dynamic]int; defer delete(queue)
	bodies_dyn: [dynamic][]int

	for start in 0 ..< n {
		if visited[start] || cell_is_land(m.cell_types[start]) do continue

		body: [dynamic]int
		clear(&queue)
		append(&queue, start)
		visited[start] = true

		for len(queue) > 0 {
			current := pop(&queue)
			append(&body, current)
			cell_to_body[current] = len(bodies_dyn)
			for nb in m.voronoi.cells[current].neighbors {
				if visited[nb] do continue
				if !cell_is_land(m.cell_types[nb]) {
					append(&queue, int(nb))
					visited[nb] = true
				}
			}
		}
		append(&bodies_dyn, body[:])
	}
	return bodies_dyn[:], cell_to_body
}

// Land cells whose perimeter is more than 55% adjacent to water → Headland.
find_headlands :: proc(m: ^VoronoiMap) {
	rocks: [dynamic]int
	defer delete(rocks)

	for ct, idx in m.cell_types {
		if !cell_is_land(ct) do continue

		land_len, water_len: f32
		for nb in m.voronoi.cells[idx].neighbors {
			s, e, ok := get_edge_between_cells(m, idx, int(nb))
			if !ok do continue
			dx, dy := s.x - e.x, s.y - e.y
			length := math.sqrt(dx * dx + dy * dy)
			if cell_is_land(m.cell_types[nb]) do land_len += length
			else do water_len += length
		}
		total := land_len + water_len
		if total > 0 && water_len / total > 0.55 do append(&rocks, idx)
	}
	for idx in rocks do m.cell_types[idx] = .Headland
}

// Land cells touching any water (and not already a headland) → Beach.
find_beaches :: proc(m: ^VoronoiMap) {
	beaches: [dynamic]int
	defer delete(beaches)

	for ct, idx in m.cell_types {
		if !cell_is_land(ct) || ct == .Headland do continue
		for nb in m.voronoi.cells[idx].neighbors {
			if !cell_is_land(m.cell_types[nb]) {
				append(&beaches, idx)
				break
			}
		}
	}
	for idx in beaches do m.cell_types[idx] = .Beach
}

// Convert land cells adjacent to two or more distinct waterbodies to water,
// so that the resulting waterbodies can be navigated end-to-end.
make_waterbodies_navigable :: proc(m: ^VoronoiMap) {
	bodies, cell_to_body := find_waterbodies(m)
	defer {
		for b in bodies do delete(b)
		delete(bodies)
		delete(cell_to_body)
	}

	seen: map[int]struct{}; defer delete(seen)
	targets: [dynamic]int; defer delete(targets)

	for ct, idx in m.cell_types {
		if !cell_is_land(ct) do continue
		clear(&seen)
		for nb in m.voronoi.cells[idx].neighbors {
			b := cell_to_body[nb]
			if b >= 0 do seen[b] = {}
		}
		if len(seen) >= 2 do append(&targets, idx)
	}
	for idx in targets do m.cell_types[idx] = .Water
}

// Classify water cells by counting land/water transitions around their ring of
// neighbours: 0 = Ocean (fully surrounded by water), 2 = Cove, more = Channel.
classify_water_cells :: proc(m: ^VoronoiMap) {
	coves, channels, oceans: [dynamic]int
	defer {delete(coves); delete(channels); delete(oceans)}

	for ct, idx in m.cell_types {
		if cell_is_land(ct) do continue
		nbs := m.voronoi.cells[idx].neighbors
		if len(nbs) == 0 do continue

		first_is_land := cell_is_land(m.cell_types[nbs[0]])
		prev_is_land := first_is_land
		transitions := 0
		for k in 1 ..< len(nbs) {
			is_land := cell_is_land(m.cell_types[nbs[k]])
			if is_land != prev_is_land do transitions += 1
			prev_is_land = is_land
		}
		if prev_is_land != first_is_land do transitions += 1

		switch transitions {
		case 0:
			append(&oceans, idx)
		case 2:
			append(&coves, idx)
		case:
			append(&channels, idx)
		}
	}
	for i in oceans do m.cell_types[i] = .Ocean
	for i in coves do m.cell_types[i] = .Cove
	for i in channels do m.cell_types[i] = .Channel
}

// Cellular-automaton smoothing. A land cell with <25% land neighbours becomes
// water; a water cell with >75% land neighbours becomes land.
voronoi_smooth :: proc(m: ^VoronoiMap, rounds: int) {
	n := len(m.cell_types)
	next := make([]CellType, n); defer delete(next)

	for _ in 0 ..< rounds {
		for i in 0 ..< n {
			land_count, water_count: int
			for nb in m.voronoi.cells[i].neighbors {
				if cell_is_land(m.cell_types[nb]) do land_count += 1
				else do water_count += 1
			}
			total := land_count + water_count
			if total == 0 {
				next[i] = m.cell_types[i]
				continue
			}
			land_frac := f32(land_count) / f32(total)
			if cell_is_land(m.cell_types[i]) {
				next[i] = .Water if land_frac < 0.25 else m.cell_types[i]
			} else {
				next[i] = .Land if land_frac > 0.75 else m.cell_types[i]
			}
		}
		m.cell_types, next = next, m.cell_types
	}
}

// Resample with a finer Poisson radius, copying cell type / height from the
// nearest source cell.
expand_init :: proc(position: [2]f32, user_data: rawptr) -> (CellType, f32) {
	src := cast(^VoronoiMap)user_data
	idx, _, _ := kdtree_nearest(src.kdtree, position)
	return src.cell_types[idx], src.heights[idx]
}

voronoi_expand :: proc(m: ^VoronoiMap, radius: f32, seed: u32) -> VoronoiMap {
	return voronoi_map_make(m.width, m.height, radius, seed, expand_init, m)
}

// =============================================================================
// Map generation
// =============================================================================

PerlinThreshold :: struct {
	seed:      i64,
	threshold: f32,
}

perlin_init :: proc(position: [2]f32, user_data: rawptr) -> (CellType, f32) {
	p := cast(^PerlinThreshold)user_data
	n := noise.noise_2d(p.seed, {f64(position[0] * 10.0), f64(position[1] * 10.0)})
	return (.Land if f32(n) > p.threshold else .Water), 0.0
}

count_land :: proc(m: ^VoronoiMap) -> int {
	c := 0
	for ct in m.cell_types do if cell_is_land(ct) do c += 1
	return c
}

generate_map :: proc(width, height: u32, seed: u32) -> VoronoiMap {
	rng := rand.create(u64(seed))
	perlin := PerlinThreshold {
		seed      = i64(seed),
		threshold = 0.0,
	}

	// First pass: tune the threshold until land coverage lands in [45%, 55%].
	l1 := voronoi_map_make(width, height, 100.0, seed, perlin_init, &perlin)
	voronoi_smooth(&l1, 2)

	percent_land := f32(count_land(&l1)) / f32(len(l1.cell_types))
	for percent_land < 0.45 || percent_land > 0.55 {
		if percent_land < 0.45 do perlin.threshold -= 0.01
		else do perlin.threshold += 0.01

		voronoi_map_destroy(&l1)
		l1 = voronoi_map_make(width, height, 100.0, seed, perlin_init, &perlin)
		voronoi_smooth(&l1, 2)
		percent_land = f32(count_land(&l1)) / f32(len(l1.cell_types))
	}

	make_waterbodies_navigable(&l1)
	classify_water_cells(&l1)

	l2 := voronoi_expand(&l1, 50.0, seed); voronoi_map_destroy(&l1)
	voronoi_smooth(&l2, 2)
	find_headlands(&l2)

	l3 := voronoi_expand(&l2, 20.0, seed); voronoi_map_destroy(&l2)
	voronoi_smooth(&l3, 2)
	find_headlands(&l3)

	l4 := voronoi_expand(&l3, 5.0, seed); voronoi_map_destroy(&l3)
	voronoi_smooth(&l4, 2)
	find_beaches(&l4)

	// Distance-to-coast and distance-to-headland fields (BFS-ish, with random
	// jitter at increasing levels).
	n := len(l4.positions)
	dist_coast_known := make([]bool, n); defer delete(dist_coast_known)
	dist_coast := make([]u32, n); defer delete(dist_coast)
	dist_headland_known := make([]bool, n); defer delete(dist_headland_known)
	dist_headland := make([]u32, n); defer delete(dist_headland)

	// Seed: coast cells are land cells that border water (or vice versa).
	for ct, idx in l4.cell_types {
		has_land := cell_is_land(ct)
		has_water := !cell_is_land(ct)
		for nb in l4.voronoi.cells[idx].neighbors {
			if cell_is_land(l4.cell_types[nb]) do has_land = true
			else do has_water = true
		}
		if has_land && has_water {
			dist_coast_known[idx] = true
			dist_coast[idx] = 0
		}
	}
	// Seed: headland cells.
	for ct, idx in l4.cell_types {
		if ct == .Headland {
			dist_headland_known[idx] = true
			dist_headland[idx] = 0
		}
	}

	for level in u32(0) ..< u32(100) {
		run_again := false
		for idx in 0 ..< n {
			// Coast distance
			if !dist_coast_known[idx] {
				run_again = true
				has_calc, has_uncalc := false, false
				for nb in l4.voronoi.cells[idx].neighbors {
					if dist_coast_known[nb] {
						if dist_coast[nb] == level {has_calc = true; break}
					} else {
						has_uncalc = true
					}
				}
				if has_calc {
					dist_coast_known[idx] = true
					if rand.float32_range(0.0, 1.0) < (1.0 - f32(level) / 50.0) {
						dist_coast[idx] = level + 1
					} else {
						dist_coast[idx] = level
					}
				} else if !has_uncalc {
					dist_coast_known[idx] = true
					dist_coast[idx] = level
				}
			}

			// Headland distance
			if !dist_headland_known[idx] {
				run_again = true
				has_calc, has_uncalc := false, false
				for nb in l4.voronoi.cells[idx].neighbors {
					if dist_headland_known[nb] {
						if dist_headland[nb] == level {has_calc = true; break}
					} else {
						has_uncalc = true
					}
				}
				if has_calc {
					dist_headland_known[idx] = true
					if rand.float32_range(0.0, 1.0) < (1.0 - f32(level) / 50.0) {
						dist_headland[idx] = level + 1
					} else {
						dist_headland[idx] = level
					}
				} else if !has_uncalc {
					dist_headland_known[idx] = true
					dist_headland[idx] = level
				}
			}
		}
		if !run_again do break
	}

	// Final heights.
	for i in 0 ..< n {
		ct := l4.cell_types[i]
		dc := f32(dist_coast[i])
		dh := f32(dist_headland[i])

		if cell_is_land(ct) {
			standard_step := dc
			headland_step := 3.0 + max(dc, 3.0 - dc)
			blend := max(1.0 - dh / 10.0, 0.0)
			l4.heights[i] =
				math.log2(1.0 + standard_step + (headland_step - standard_step) * blend) * 5000.0
		} else {
			l4.heights[i] = -math.log2(1.0 + dc) * 2000.0
		}
	}

	return l4
}

// =============================================================================
// Image output
// =============================================================================

color_for :: proc(ct: CellType) -> (r, g, b: u8) {
	switch ct {
	case .Land:
		return 0, 255, 0
	case .Water:
		return 0, 0, 255
	case .Headland:
		return 200, 200, 200
	case .Beach:
		return 255, 255, 0
	case .Cove:
		return 179, 236, 255
	case .Channel:
		return 43, 176, 237
	case .Ocean:
		return 3, 83, 136
	}
	return 0, 0, 0
}

// Per-pixel nearest-cell rasterisation (replaces the imageproc polygon draw).
draw_image_rgb :: proc(m: ^VoronoiMap, width, height: u32) -> []u8 {
	img := make([]u8, int(width) * int(height) * 3)
	for y in 0 ..< height {
		for x in 0 ..< width {
			sx := f32(x) / f32(width) * f32(m.width)
			sy := f32(y) / f32(height) * f32(m.height)
			idx, _, _ := kdtree_nearest(m.kdtree, {sx, sy})
			r, g, b := color_for(m.cell_types[idx])
			i := (int(y) * int(width) + int(x)) * 3
			img[i + 0] = r
			img[i + 1] = g
			img[i + 2] = b
		}
	}
	return img
}

draw_heightmap_u16 :: proc(m: ^VoronoiMap, width, height: u32) -> []u16 {
	img := make([]u16, int(width) * int(height))
	for y in 0 ..< height {
		for x in 0 ..< width {
			sx := f32(x) / f32(width) * f32(m.width)
			sy := f32(y) / f32(height) * f32(m.height)
			idx, _, _ := kdtree_nearest(m.kdtree, {sx, sy})
			SEA_LEVEL :: 20000.0
			v := SEA_LEVEL + m.heights[idx]
			v = clamp(v, 0.0, 65535.0)
			img[int(y) * int(width) + int(x)] = u16(v)
		}
	}
	return img
}

// Watermap simulation. Note: the input `surface_water_next` accumulator was a
// straight port of the original, which never zeroes itself between iterations
// — kept that way deliberately so behaviour matches.
draw_watermap_rgb :: proc(m: ^VoronoiMap, width, height: u32) -> []u8 {
	n := len(m.positions)
	surface_water := make([]f32, n); defer delete(surface_water)
	surface_water_next := make([]f32, n); defer delete(surface_water_next)
	water_flow := make([]f32, n); defer delete(water_flow)

	for _ in 0 ..< 100 {
		for i in 0 ..< n do surface_water[i] += 1.0

		for i in 0 ..< n {
			if surface_water[i] < 0.1 do continue

			lower := 0
			for nb in m.voronoi.cells[i].neighbors {
				if m.heights[nb] < m.heights[i] do lower += 1
			}
			if lower == 0 do continue

			share := surface_water[i] / f32(lower)
			for nb in m.voronoi.cells[i].neighbors {
				if m.heights[nb] < m.heights[i] {
					surface_water_next[nb] += share
					water_flow[nb] += share
				}
			}
			surface_water_next[i] = 0.0
		}
		copy(surface_water, surface_water_next)
	}

	img := make([]u8, int(width) * int(height) * 3)
	for y in 0 ..< height {
		for x in 0 ..< width {
			sx := f32(x) / f32(width) * f32(m.width)
			sy := f32(y) / f32(height) * f32(m.height)
			idx, _, _ := kdtree_nearest(m.kdtree, {sx, sy})
			if m.heights[idx] < 0.0 {
				v := i32(water_flow[idx] * 100.0)
				v = clamp(v, 0, 255)
				k := (int(y) * int(width) + int(x)) * 3
				img[k + 2] = u8(v)
			}
		}
	}
	return img
}

// =============================================================================
// Entry point
// =============================================================================

gemmapmain :: proc() {
	width, height: u32 = 2000, 2000
	seed: u32 = 0xCAFEF00D

	m := generate_map(width, height, seed)
	defer voronoi_map_destroy(&m)

	rgb := draw_image_rgb(&m, width, height); defer delete(rgb)
	rgb_img := rl.Image {
		data    = raw_data(rgb),
		width   = i32(width),
		height  = i32(height),
		mipmaps = 1,
		format  = .UNCOMPRESSED_R8G8B8,
	}
	if !rl.ExportImage(rgb_img, "map.png") {
		fmt.eprintln("failed to write map.png")
	}

	water := draw_watermap_rgb(&m, width, height); defer delete(water)
	water_img := rl.Image {
		data    = raw_data(water),
		width   = i32(width),
		height  = i32(height),
		mipmaps = 1,
		format  = .UNCOMPRESSED_R8G8B8,
	}
	if !rl.ExportImage(water_img, "water.png") {
		fmt.eprintln("failed to write water.png")
	}

	// 16-bit grayscale PGM is the easiest way to round-trip the heightmap;
	// raylib's PNG exporter is 8-bit only.
	hm := draw_heightmap_u16(&m, width, height); defer delete(hm)
	write_pgm_p5("height.pgm", width, height, hm)

	// 8-bit grayscale PNG preview (high byte of each u16).
	hm8 := make([]u8, len(hm)); defer delete(hm8)
	for v, i in hm do hm8[i] = u8(v >> 8)
	hm_img := rl.Image{
		data    = raw_data(hm8),
		width   = i32(width),
		height  = i32(height),
		mipmaps = 1,
		format  = .UNCOMPRESSED_GRAYSCALE,
	}
	if !rl.ExportImage(hm_img, "height.png") {
		fmt.eprintln("failed to write height.png")
	}
}

write_pgm_p5 :: proc(path: string, w, h: u32, data: []u16) {
	f, err := os.open(path, os.O_WRONLY | os.O_CREATE | os.O_TRUNC)
	if err != nil {
		fmt.eprintfln("open %s failed: %v", path, err)
		return
	}
	defer os.close(f)
	header := fmt.tprintf("P5\n%d %d\n65535\n", w, h)
	os.write_string(f, header)
	// PGM 16-bit is big-endian.
	buf := make([]u8, len(data) * 2); defer delete(buf)
	for v, i in data {
		buf[2 * i + 0] = u8(v >> 8)
		buf[2 * i + 1] = u8(v & 0xFF)
	}
	os.write(f, buf)
}

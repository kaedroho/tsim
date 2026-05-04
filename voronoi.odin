package main

import "base:runtime"

VoronoiCell :: struct {
	site:      Vector2,
	vertices:  []Vector2, // CCW polygon, clipped to [0, width] x [0, height]
	neighbors: []u32, // adjacent real-site indices, CCW (no ghost neighbours)
	triangles: []u32, // incident Delaunay triangle IDs (no ghost-incident triangles)
}

Voronoi :: struct {
	cells:     []VoronoiCell,
	vertices:  []Vector2, // circumcenter per Delaunay triangle (incl. ghost-incident)
	sites:     []Vector2, // final relaxed site positions
	allocator: runtime.Allocator,
}

voronoi_build :: proc(
	sites: []Vector2,
	width: f32,
	height: f32,
	lloyd_iters: int,
	allocator := context.allocator,
) -> Voronoi {
	num_sites := len(sites)

	relaxed := make([]Vector2, num_sites, allocator)
	copy(relaxed, sites)

	// Four ghost points far outside the bbox. They become the convex hull, so
	// every real site is interior with a bounded Voronoi cell.
	extended := make([]Vector2, num_sites + 4, allocator)
	defer delete(extended, allocator)
	pad := max(width, height) * 100
	_voronoi_pack(extended, relaxed, width, height, pad)

	trn := triangulate(sites)

	for _ in 0 ..< lloyd_iters {
		_voronoi_lloyd_step(sites, &trn, relaxed, width, height, allocator)
		_voronoi_pack(extended, relaxed, width, height, pad)
		trn := triangulate(relaxed)
		copy(sites, relaxed)
	}

	return _voronoi_finalize(sites, &trn, relaxed, width, height, allocator)
}

voronoi_destroy :: proc(v: ^Voronoi) {
	for &cell in v.cells {
		delete(cell.vertices, v.allocator)
		delete(cell.neighbors, v.allocator)
		delete(cell.triangles, v.allocator)
	}
	delete(v.cells, v.allocator)
	delete(v.vertices, v.allocator)
	delete(v.sites, v.allocator)
	v^ = {}
}

// ---------------------------------------------------------------------------
// internals
// ---------------------------------------------------------------------------

@(private)
_voronoi_pack :: proc(extended, real_sites: []Vector2, width, height, pad: f32) {
	num_real := len(real_sites)
	copy(extended[:num_real], real_sites)
	extended[num_real + 0] = {-pad, -pad}
	extended[num_real + 1] = {width + pad, -pad}
	extended[num_real + 2] = {width + pad, height + pad}
	extended[num_real + 3] = {-pad, height + pad}
}

// Circumcenter per Delaunay triangle (parallel to trn.triangles in groups of 3).
@(private)
_voronoi_circumcenters :: proc(
	sites: []Vector2,
	trn: ^Triangulation,
	allocator: runtime.Allocator,
) -> []Vector2 {
	num_tri := len(trn.triangles) / 3
	out := make([]Vector2, num_tri, allocator)
	for t in 0 ..< num_tri {
		a := sites[trn.triangles[3 * t]]
		b := sites[trn.triangles[3 * t + 1]]
		c := sites[trn.triangles[3 * t + 2]]
		out[t] = circumcenter(a, b, c)
	}
	return out
}

// inedges[s] is some half-edge ending at site s. Hull edges are preferred so
// hull walks terminate at the boundary; with ghost points there should be no
// real hull sites, but we keep this for robustness.
@(private)
_voronoi_inedges :: proc(
	sites: []Vector2,
	trn: ^Triangulation,
	allocator: runtime.Allocator,
) -> []i32 {
	n := len(sites)
	out := make([]i32, n, allocator)
	for i in 0 ..< n do out[i] = -1
	for e in 0 ..< len(trn.triangles) {
		s := int(trn.triangles[_voronoi_next_he(e)])
		if trn.halfedges[e] == EMPTY || out[s] == -1 {
			out[s] = i32(e)
		}
	}
	return out
}

@(private)
_voronoi_next_he :: proc(e: int) -> int {
	return e - 2 if e % 3 == 2 else e + 1
}

// Walk the triangles incident to `site` in CCW order. Produces parallel arrays:
// `tris_out[k]` = incident triangle ID, `nbrs_out[k]` = neighbour vertex shared
// with the previous triangle in the walk.
@(private)
_voronoi_walk_cell :: proc(
	trn: ^Triangulation,
	inedges: []i32,
	site: int,
	tris_out, nbrs_out: ^[dynamic]u32,
) {
	clear(tris_out)
	clear(nbrs_out)
	if inedges[site] == -1 do return

	e0 := int(inedges[site])
	e := e0
	for {
		append(tris_out, u32(e / 3))
		append(nbrs_out, trn.triangles[e])

		e_next := _voronoi_next_he(e)
		if int(trn.triangles[e_next]) != site do break
		twin := trn.halfedges[e_next]
		if twin == EMPTY do break
		e = int(twin)
		if e == e0 do break
	}
}

@(private)
_voronoi_lloyd_step :: proc(
	sites: []Vector2,
	trn: ^Triangulation,
	relaxed: []Vector2,
	width, height: f32,
	allocator: runtime.Allocator,
) {
	circ := _voronoi_circumcenters(sites, trn, allocator)
	defer delete(circ, allocator)
	inedges := _voronoi_inedges(sites, trn, allocator)
	defer delete(inedges, allocator)

	tris: [dynamic]u32
	nbrs: [dynamic]u32
	poly_a, poly_b, poly_c: [dynamic]Vector2
	defer {
		delete(tris)
		delete(nbrs)
		delete(poly_a)
		delete(poly_b)
		delete(poly_c)
	}

	for i in 0 ..< len(relaxed) {
		_voronoi_walk_cell(trn, inedges, i, &tris, &nbrs)
		clear(&poly_a)
		for t in tris do append(&poly_a, circ[t])

		clipped := _voronoi_clip_rect(&poly_a, &poly_b, &poly_c, 0, 0, width, height)
		if len(clipped) >= 3 {
			relaxed[i] = _voronoi_centroid(clipped)
		}
	}
}

@(private)
_voronoi_finalize :: proc(
	sites: []Vector2,
	trn: ^Triangulation,
	relaxed: []Vector2,
	width, height: f32,
	allocator: runtime.Allocator,
) -> Voronoi {
	num_sites := len(relaxed)

	circ := _voronoi_circumcenters(sites, trn, allocator)
	inedges := _voronoi_inedges(sites, trn, allocator)
	defer delete(inedges, allocator)

	cells := make([]VoronoiCell, num_sites, allocator)

	tris: [dynamic]u32
	nbrs: [dynamic]u32
	poly_a, poly_b, poly_c: [dynamic]Vector2
	tris_real: [dynamic]u32
	nbrs_real: [dynamic]u32
	defer {
		delete(tris)
		delete(nbrs)
		delete(poly_a)
		delete(poly_b)
		delete(poly_c)
		delete(tris_real)
		delete(nbrs_real)
	}

	for i in 0 ..< num_sites {
		_voronoi_walk_cell(trn, inedges, i, &tris, &nbrs)

		// Clip the raw polygon to the bbox.
		clear(&poly_a)
		for t in tris do append(&poly_a, circ[t])
		clipped := _voronoi_clip_rect(&poly_a, &poly_b, &poly_c, 0, 0, width, height)

		// Filter out ghost-incident triangles and ghost neighbours.
		clear(&tris_real)
		clear(&nbrs_real)
		for k in 0 ..< len(tris) {
			tri_id := tris[k]
			a := int(trn.triangles[3 * tri_id])
			b := int(trn.triangles[3 * tri_id + 1])
			c := int(trn.triangles[3 * tri_id + 2])
			if a >= num_sites || b >= num_sites || c >= num_sites do continue
			append(&tris_real, tri_id)
			if int(nbrs[k]) < num_sites do append(&nbrs_real, nbrs[k])
		}

		vbuf := make([]Vector2, len(clipped), allocator)
		copy(vbuf, clipped)
		tbuf := make([]u32, len(tris_real), allocator)
		copy(tbuf, tris_real[:])
		nbuf := make([]u32, len(nbrs_real), allocator)
		copy(nbuf, nbrs_real[:])

		cells[i] = VoronoiCell {
			site      = relaxed[i],
			vertices  = vbuf,
			triangles = tbuf,
			neighbors = nbuf,
		}
	}

	return Voronoi{cells = cells, vertices = circ, sites = relaxed, allocator = allocator}
}

// ---------------------------------------------------------------------------
// polygon helpers
// ---------------------------------------------------------------------------

// Sutherland-Hodgman clip of `input` against axis-aligned rect [x0..x1] x [y0..y1].
// Uses `tmp_a` and `tmp_b` as ping-pong scratch buffers; returns a slice into
// the buffer holding the final result (one of the three).
@(private)
_voronoi_clip_rect :: proc(
	input, tmp_a, tmp_b: ^[dynamic]Vector2,
	x0, y0, x1, y1: f32,
) -> []Vector2 {
	_voronoi_clip_edge(input, tmp_a, 0, +1, x0)
	_voronoi_clip_edge(tmp_a, tmp_b, 0, -1, x1)
	_voronoi_clip_edge(tmp_b, tmp_a, 1, +1, y0)
	_voronoi_clip_edge(tmp_a, tmp_b, 1, -1, y1)
	return tmp_b[:]
}

@(private)
_voronoi_clip_edge :: proc(src, dst: ^[dynamic]Vector2, axis: int, sign, limit: f32) {
	clear(dst)
	pts := src[:]
	if len(pts) == 0 do return

	prev := pts[len(pts) - 1]
	prev_in := sign * (prev[axis] - limit) >= 0
	for cur in pts {
		cur_in := sign * (cur[axis] - limit) >= 0
		if cur_in {
			if !prev_in do append(dst, _voronoi_axis_intersect(prev, cur, axis, limit))
			append(dst, cur)
		} else if prev_in {
			append(dst, _voronoi_axis_intersect(prev, cur, axis, limit))
		}
		prev = cur
		prev_in = cur_in
	}
}

@(private)
_voronoi_axis_intersect :: proc(p, q: Vector2, axis: int, limit: f32) -> Vector2 {
	denom := q[axis] - p[axis]
	if denom == 0 do return p
	t := (limit - p[axis]) / denom
	return p + t * (q - p)
}

// Centroid via the signed-area formula. Falls back to the first vertex for
// degenerate inputs (zero or near-zero area).
@(private)
_voronoi_centroid :: proc(pts: []Vector2) -> Vector2 {
	if len(pts) == 0 do return {0, 0}
	if len(pts) < 3 do return pts[0]

	a2: f32 = 0
	cx, cy: f32 = 0, 0
	for i in 0 ..< len(pts) {
		p := pts[i]
		q := pts[(i + 1) % len(pts)]
		cross := p.x * q.y - q.x * p.y
		a2 += cross
		cx += (p.x + q.x) * cross
		cy += (p.y + q.y) * cross
	}
	if a2 == 0 do return pts[0]
	return {cx / (3 * a2), cy / (3 * a2)}
}

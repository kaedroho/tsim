package main

import "core:fmt"
import rl "vendor:raylib"

MAP_WIDTH, MAP_HEIGHT :: 2000, 2000
MAP_SEED :: 0xCAFEF00D

SCALE :: f32(0.5) // world is 2000x2000, window is 1000x1000

main :: proc() {
	m := generate_map(MAP_WIDTH, MAP_HEIGHT, MAP_SEED)
	defer map_destroy(&m)

	rgb := draw_image_rgb(&m.geology, MAP_WIDTH, MAP_HEIGHT); defer delete(rgb)
	rgb_img := rl.Image {
		data    = raw_data(rgb),
		width   = i32(MAP_WIDTH),
		height  = i32(MAP_HEIGHT),
		mipmaps = 1,
		format  = .UNCOMPRESSED_R8G8B8,
	}
	if !rl.ExportImage(rgb_img, "map.png") {
		fmt.eprintln("failed to write map.png")
	}

	water := draw_watermap_rgb(&m.geology, MAP_WIDTH, MAP_HEIGHT); defer delete(water)
	water_img := rl.Image {
		data    = raw_data(water),
		width   = i32(MAP_WIDTH),
		height  = i32(MAP_HEIGHT),
		mipmaps = 1,
		format  = .UNCOMPRESSED_R8G8B8,
	}
	if !rl.ExportImage(water_img, "water.png") {
		fmt.eprintln("failed to write water.png")
	}

	// 16-bit grayscale PGM is the easiest way to round-trip the heightmap;
	// raylib's PNG exporter is 8-bit only.
	hm := draw_heightmap_u16(&m.geology, MAP_WIDTH, MAP_HEIGHT); defer delete(hm)
	write_pgm_p5("height.pgm", MAP_WIDTH, MAP_HEIGHT, hm)

	// 8-bit grayscale PNG preview (high byte of each u16).
	hm8 := make([]u8, len(hm)); defer delete(hm8)
	for v, i in hm do hm8[i] = u8(v >> 8)
	hm_img := rl.Image {
		data    = raw_data(hm8),
		width   = i32(MAP_WIDTH),
		height  = i32(MAP_HEIGHT),
		mipmaps = 1,
		format  = .UNCOMPRESSED_GRAYSCALE,
	}
	if !rl.ExportImage(hm_img, "height.png") {
		fmt.eprintln("failed to write height.png")
	}

	is_neighbor := make([]bool, len(m.geology.voronoi.cells))
	defer delete(is_neighbor)

	rl.InitWindow(1000, 1000, "TSim")
	defer rl.CloseWindow()


	for !rl.WindowShouldClose() {
		nearest, _, _ := kdtree_nearest(m.geology.kdtree, rl.GetMousePosition() * 2)

		// Refresh neighbour mask for the nearest cell.
		for i in 0 ..< len(is_neighbor) do is_neighbor[i] = false
		for nb in m.geology.voronoi.cells[nearest].neighbors {
			is_neighbor[nb] = true
		}

		rl.BeginDrawing()
		rl.ClearBackground(rl.BLUE)

		fps := rl.GetFPS()
		rl.DrawText(fmt.ctprint("FPS: ", fps), 10, 10, 20, rl.BLACK)

		rl.EndDrawing()
		free_all(context.temp_allocator)
	}
}

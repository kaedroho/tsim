package main

import "core:fmt"
import rl "vendor:raylib"

main :: proc() {
	positions := poisson_generate(2000, 2000, 100, 0x12345678)

	gemmapmain()

	voronoi := voronoi_build(positions[:], 2000, 2000, 5)
	defer voronoi_destroy(&voronoi)

	// Re-triangulate the relaxed sites for the Delaunay overlay.
	trn := triangulate(voronoi.sites)

	// kdtree on the relaxed sites so mouse-nearest matches the drawn cells.
	kdtree := kdtree_make(voronoi.sites)
	defer kdtree_destroy(&kdtree)

	is_neighbor := make([]bool, len(voronoi.cells))
	defer delete(is_neighbor)

	rl.InitWindow(1000, 1000, "TSim")
	defer rl.CloseWindow()

	SCALE :: f32(0.5) // world is 2000x2000, window is 1000x1000

	for !rl.WindowShouldClose() {
		nearest, _, _ := kdtree_nearest(kdtree, rl.GetMousePosition() * 2)

		// Refresh neighbour mask for the nearest cell.
		for i in 0 ..< len(is_neighbor) do is_neighbor[i] = false
		for nb in voronoi.cells[nearest].neighbors {
			is_neighbor[nb] = true
		}

		rl.BeginDrawing()
		rl.ClearBackground(rl.BLUE)

		// Filled cells.
		for cell, idx in voronoi.cells {
			if len(cell.vertices) < 3 do continue
			color: rl.Color
			switch {
			case u32(idx) == nearest:
				color = rl.RED
			case is_neighbor[idx]:
				color = rl.YELLOW
			case:
				color = rl.LIGHTGRAY
			}
			pts := make([]rl.Vector2, len(cell.vertices), context.temp_allocator)
			for v, i in cell.vertices do pts[i] = {v.x * SCALE, v.y * SCALE}
			rl.DrawTriangleFan(raw_data(pts), i32(len(pts)), color)
		}

		// Cell edges drawn on top.
		for cell in voronoi.cells {
			for i in 0 ..< len(cell.vertices) {
				a := cell.vertices[i]
				b := cell.vertices[(i + 1) % len(cell.vertices)]
				rl.DrawLineV({a.x * SCALE, a.y * SCALE}, {b.x * SCALE, b.y * SCALE}, rl.BLACK)
			}
		}

		// Delaunay overlay (each edge drawn twice; cheap and visually fine).
		for t in 0 ..< len(trn.triangles) / 3 {
			a := voronoi.sites[trn.triangles[3 * t]]
			b := voronoi.sites[trn.triangles[3 * t + 1]]
			c := voronoi.sites[trn.triangles[3 * t + 2]]
			rl.DrawLineV({a.x * SCALE, a.y * SCALE}, {b.x * SCALE, b.y * SCALE}, rl.DARKGREEN)
			rl.DrawLineV({b.x * SCALE, b.y * SCALE}, {c.x * SCALE, c.y * SCALE}, rl.DARKGREEN)
			rl.DrawLineV({c.x * SCALE, c.y * SCALE}, {a.x * SCALE, a.y * SCALE}, rl.DARKGREEN)
		}

		// Sites.
		for pos, idx in voronoi.sites {
			color := rl.BLACK if u32(idx) == nearest else rl.WHITE
			radius: f32 = 6 if u32(idx) == nearest else 4
			rl.DrawCircle(i32(pos.x * SCALE), i32(pos.y * SCALE), radius, color)
		}

		fps := rl.GetFPS()
		rl.DrawText(fmt.ctprint("FPS: ", fps), 10, 10, 20, rl.BLACK)

		rl.EndDrawing()
		free_all(context.temp_allocator)
	}
}

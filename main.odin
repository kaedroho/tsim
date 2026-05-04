package main

import "core:fmt"
import rl "vendor:raylib"

main :: proc() {
	positions := poisson_generate(2000, 2000, 100, 0x12345678)

	gemmapmain()

	kdtree := kdtree_make(positions[:])
	voronoi := voronoi_build(positions[:], 2000, 2000, 5)

	rl.InitWindow(1000, 1000, "TSim")
	defer rl.CloseWindow()

	for !rl.WindowShouldClose() {
		nearest, _, _ := kdtree_nearest(kdtree, rl.GetMousePosition() * 2)

		rl.BeginDrawing()
		rl.ClearBackground(rl.BLUE)

		for pos, idx in positions {
			rl.DrawCircle(
				i32(pos.x / 2),
				i32(pos.y / 2),
				5,
				rl.BLACK if u32(idx) == nearest else rl.WHITE,
			)
		}

		fps := rl.GetFPS()
		rl.DrawText(fmt.ctprint("FPS: ", fps), 10, 10, 20, rl.BLACK)

		rl.EndDrawing()
		free_all(context.temp_allocator)
	}
}

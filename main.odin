package main

import "core:math/rand"
import vmem "core:mem/virtual"
import "core:fmt"
import rl "vendor:raylib"

main :: proc() {
	positions := poisson_generate(1000, 1000, 30, 0x12345678)

	arena: vmem.Arena
	arena_allocator := vmem.arena_allocator(&arena)
	kdtree := kdtree_build(positions[:])

	rl.InitWindow(1000, 1000, "TSim")
	defer rl.CloseWindow()

	for !rl.WindowShouldClose() {
		nearest, _, _ := kdtree_nearest(kdtree, rl.GetMousePosition())

		rl.BeginDrawing()
		rl.ClearBackground(rl.BLUE)

		for pos, idx in positions {
			rl.DrawCircle(i32(pos.x), i32(pos.y), 5, rl.BLACK if u32(idx) == nearest else rl.WHITE)
		}

		fps := rl.GetFPS()
		rl.DrawText(fmt.ctprint("FPS: ", fps), 10, 10, 20, rl.BLACK);

		rl.EndDrawing()
		free_all(context.temp_allocator)
	}
}

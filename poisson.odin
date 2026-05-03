package main

import "base:runtime"
import "core:math"
import "core:math/rand"

poisson_generate :: proc(width, height, radius: f32, seed: u64) -> [dynamic][2]f32 {
	K :: 30

	// Local PRNG state — context modifications don't escape this procedure.
	rng_state: runtime.Default_Random_State
	context.random_generator = runtime.default_random_generator(&rng_state)
	rand.reset(seed)

	// Background grid: cell size = r/√2 guarantees at most 1 sample per cell.
	cell_size := radius / math.sqrt(f32(2))
	grid_w := int(math.ceil(width / cell_size))
	grid_h := int(math.ceil(height / cell_size))

	grid := make([]int, grid_w * grid_h)
	defer delete(grid)
	for i in 0 ..< len(grid) do grid[i] = -1

	points: [dynamic][2]f32
	active: [dynamic]int
	defer delete(active)

	// Step 1 — seed with a random initial sample.
	{
		x := rand.float32() * width
		y := rand.float32() * height
		append(&points, [2]f32{f32(x), f32(y)})
		append(&active, 0)
		grid[int(y / cell_size) * grid_w + int(x / cell_size)] = 0
	}

	r2 := radius * radius

	// Step 2 — keep generating until no active samples remain.
	for len(active) > 0 {
		ai := int(rand.float32() * f32(len(active)))
		if ai >= len(active) do ai = len(active) - 1
		center := points[active[ai]]

		accepted := false
		attempts: for _ in 0 ..< K {
			// Uniform sample in the annulus [r, 2r] around center.
			angle := rand.float32() * 2.0 * math.PI
			dist := radius * (1.0 + rand.float32())
			cx := f32(center.x) + dist * math.cos(angle)
			cy := f32(center.y) + dist * math.sin(angle)

			if cx < 0 || cx >= width || cy < 0 || cy >= height do continue

			cgx := int(cx / cell_size)
			cgy := int(cy / cell_size)
			x0 := max(0, cgx - 2)
			x1 := min(grid_w - 1, cgx + 2)
			y0 := max(0, cgy - 2)
			y1 := min(grid_h - 1, cgy + 2)

			// Reject if any neighbour sample is closer than r.
			for ny in y0 ..= y1 {
				for nx in x0 ..= x1 {
					pi := grid[ny * grid_w + nx]
					if pi < 0 do continue
					dx := f32(points[pi].x) - cx
					dy := f32(points[pi].y) - cy
					if dx * dx + dy * dy < r2 do continue attempts
				}
			}

			// Accept candidate.
			append(&points, [2]f32{f32(cx), f32(cy)})
			new_idx := len(points) - 1
			grid[cgy * grid_w + cgx] = new_idx
			append(&active, new_idx)
			accepted = true
			break
		}

		if !accepted {
			// Swap-remove: O(1) deletion from active list.
			active[ai] = active[len(active) - 1]
			pop(&active)
		}
	}

	return points
}

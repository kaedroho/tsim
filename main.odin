package main

import "core:fmt"
import "core:nbio"
import sdl "vendor:sdl3"

MAP_WIDTH, MAP_HEIGHT :: 2000, 2000
MAP_SEED :: 0xCAFEF00D

load_map :: proc() {
	m := generate_map(MAP_WIDTH, MAP_HEIGHT, MAP_SEED)
	defer map_destroy(&m)
}

main :: proc() {
	// Setup I/O
	if err := nbio.acquire_thread_event_loop(); err != nil {
		fmt.eprintln("acquire_thread_event_loop:", nbio.error_string(err))
		return
	}
	defer nbio.release_thread_event_loop()

	// Setup SDL
	sdl.SetLogPriorities(.VERBOSE)
	meta_ok := sdl.SetAppMetadata("TSim", "0.1", "uk.kaed.tsim")
	sdl_ok := sdl.Init({.VIDEO})
	if !meta_ok || !sdl_ok {
		fmt.eprintln("Failed to initialize SDL:", sdl.GetError())
		return
	}
	defer sdl.Quit()

	// Setup window
	window := sdl.CreateWindow("TSim", 1000, 1000, nil)
	if window == nil {
		fmt.eprintln("Failed to create window:", sdl.GetError())
		return
	}
	defer sdl.DestroyWindow(window)

	// Setup GPU device
	gpu := sdl.CreateGPUDevice({.METALLIB}, true, nil)
	if gpu == nil {
		fmt.eprintln("Failed to create GPU device:", sdl.GetError())
		return
	}
	defer sdl.DestroyGPUDevice(gpu)
	ok := sdl.ClaimWindowForGPUDevice(gpu, window)
	if !ok {
		fmt.eprintln("Failed to claim window for GPU device:", sdl.GetError())
		return
	}

	// Create debug window
	debug_init()
	defer debug_fini()

	main_loop: for {
		frame_start := sdl.GetTicksNS()

		// I/O Poll
		nbio.tick()

		// SDL events
		for e: sdl.Event; sdl.PollEvent(&e); {
			#partial switch e.type {
			case .QUIT:
				break main_loop
			case .WINDOW_CLOSE_REQUESTED:
				break main_loop
			}
		}

		// Render
		cmd_buf := sdl.AcquireGPUCommandBuffer(gpu)
		swapchain_tex: ^sdl.GPUTexture
		ok = sdl.WaitAndAcquireGPUSwapchainTexture(
			cmd_buf,
			window,
			&swapchain_tex,
			nil,
			nil,
		); assert(ok)
		color_target := sdl.GPUColorTargetInfo {
			texture     = swapchain_tex,
			load_op     = .CLEAR,
			clear_color = {0, 0.2, 0.4, 1},
			store_op    = .STORE,
		}
		render_pass := sdl.BeginGPURenderPass(cmd_buf, &color_target, 1, nil)
		// Draw here
		sdl.EndGPURenderPass(render_pass)

		ok = sdl.SubmitGPUCommandBuffer(cmd_buf); assert(ok)

		// Debug render
		debug_render()

		free_all(context.temp_allocator)
	}
}

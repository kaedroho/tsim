package main

import vmem "core:mem/virtual"
import "core:slice"

Vector2 :: [2]f32

KdTreeNode :: struct {
	idx:   u32,
	pos:   Vector2,
	left:  ^KdTreeNode,
	right: ^KdTreeNode,
}

KdTree :: struct {
	root:  ^KdTreeNode,
	arena: vmem.Arena,
}

@(private = "file")
BuildPoint :: struct {
	idx: u32,
	pos: Vector2,
}

@(private = "file")
kdtree_build_node :: proc(
	points: []BuildPoint,
	depth: u32 = 0,
	allocator := context.allocator,
) -> ^KdTreeNode {
	if len(points) == 0 {
		return nil
	}

	if depth % 2 == 0 {
		slice.sort_by(points, proc(a, b: BuildPoint) -> bool {return a.pos.x < b.pos.x})
	} else {
		slice.sort_by(points, proc(a, b: BuildPoint) -> bool {return a.pos.y < b.pos.y})
	}

	median := len(points) / 2

	node := new(KdTreeNode, allocator)
	node.idx = points[median].idx
	node.pos = points[median].pos
	node.left = kdtree_build_node(points[:median], depth + 1, allocator)
	node.right = kdtree_build_node(points[median + 1:], depth + 1, allocator)
	return node
}

kdtree_make :: proc(points: []Vector2, allocator := context.allocator) -> KdTree {
	build_points := make([dynamic]BuildPoint, 0, len(points))
	for pos, idx in points {
		append(&build_points, BuildPoint{pos = pos, idx = u32(idx)})
	}

	arena: vmem.Arena
	arena_allocator := vmem.arena_allocator(&arena)

	return KdTree {
		root = kdtree_build_node(build_points[:], allocator = arena_allocator),
		arena = arena,
	}
}

kdtree_destroy :: proc(kd: ^KdTree) {
	vmem.arena_destroy(&kd.arena)
	kd^ = {}
}

kdtree_nearest :: proc(kd: KdTree, target: Vector2) -> (idx: u32, pos: Vector2, found: bool) {
	if kd.root == nil {
		return 0, {}, false
	}
	best := kd.root
	best_dist_sq := vec2_dist_sq(target, kd.root.pos)
	kdtree_nearest_search(kd.root, target, 0, &best, &best_dist_sq)
	return best.idx, best.pos, true
}

kdtree_nearest_search :: proc(
	node: ^KdTreeNode,
	target: Vector2,
	depth: u32,
	best: ^^KdTreeNode,
	best_dist_sq: ^f32,
) {
	if node == nil {
		return
	}

	d_sq := vec2_dist_sq(target, node.pos)
	if d_sq < best_dist_sq^ {
		best^ = node
		best_dist_sq^ = d_sq
	}

	axis := depth % 2
	diff: f32
	if axis == 0 {
		diff = target.x - node.pos.x
	} else {
		diff = target.y - node.pos.y
	}

	near, far: ^KdTreeNode
	if diff < 0 {
		near, far = node.left, node.right
	} else {
		near, far = node.right, node.left
	}

	kdtree_nearest_search(near, target, depth + 1, best, best_dist_sq)

	// Only recurse into the far side if the splitting plane is closer
	// than the best distance found so far.
	if diff * diff < best_dist_sq^ {
		kdtree_nearest_search(far, target, depth + 1, best, best_dist_sq)
	}
}

vec2_dist_sq :: proc(a, b: Vector2) -> f32 {
	dx := a.x - b.x
	dy := a.y - b.y
	return dx * dx + dy * dy
}

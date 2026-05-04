// Port of mapbox/delaunator (https://github.com/mapbox/delaunator) to Odin.
//
// A fast library for Delaunay triangulation of 2-D points. After running
// `init`, the following slices are populated:
//
//   triangles : flat array of triangle vertex indices, every group of 3
//               forms one CCW triangle.
//   halfedges : halfedges[i] is the index of the twin half-edge in the
//               adjacent triangle, or -1 for boundary half-edges.
//   hull      : indices of points on the convex hull, counter-clockwise.
//
// Call `update` to re-triangulate after mutating `coords` in place (useful
// for Lloyd relaxation and other iterative algorithms).
//
// NOTE: the upstream JS uses `robust-predicates` for adaptive-precision
// orientation tests. Porting that whole library is out of scope; the
// `orient2d` here is a plain determinant. It is fast and works for
// well-behaved input but can misclassify near-collinear edge cases.
// Swap it out if you need bullet-proof guarantees.
package main

import "base:runtime"
import "core:math"

EPSILON :: 1.1920928955078125e-7 // Math.pow(2, -23)
EDGE_STACK_SIZE :: 512

Triangulation :: struct {
	coords:        [][2]f32,

	// Outputs (sub-slices into the backing buffers below).
	triangles:     []u32,
	halfedges:     []i32,
	hull:          []u32,

	// Backing buffers.
	_triangles:    []u32,
	_halfedges:    []i32,
	_hull_buf:     []u32,

	// Hull tracking.
	_hash_size:    int,
	_hull_prev:    []u32,
	_hull_next:    []u32,
	_hull_tri:     []u32,
	_hull_hash:    []i32,

	// Sorting helpers.
	_ids:          []u32,
	_dists:        []f32,
	triangles_len: int,
	_cx:           f32,
	_cy:           f32,
	_hull_start:   u32,
	allocator:     runtime.Allocator,
}

// init runs the triangulation on a `[][2]f32` coordinate array. The slice
// is borrowed - it must outlive the Delaunator (or be mutated in-place
// when calling `update`).
delaunay_init :: proc(trn: ^Triangulation, coords: [][2]f32, allocator := context.allocator) {
	trn.allocator = allocator
	trn.coords = coords
	_alloc_buffers(trn)
	delaunay_update(trn)
}

delaunay_destroy :: proc(trn: ^Triangulation) {
	delete(trn._triangles, trn.allocator)
	delete(trn._halfedges, trn.allocator)
	delete(trn._hull_prev, trn.allocator)
	delete(trn._hull_next, trn.allocator)
	delete(trn._hull_tri, trn.allocator)
	delete(trn._hull_hash, trn.allocator)
	delete(trn._hull_buf, trn.allocator)
	delete(trn._ids, trn.allocator)
	delete(trn._dists, trn.allocator)
	trn^ = {}
}

@(private)
_alloc_buffers :: proc(trn: ^Triangulation) {
	num_coords := len(trn.coords)
	max_triangles := max(2 * num_coords - 5, 0)
	allocator := trn.allocator

	trn._triangles = make([]u32, max_triangles * 3, allocator)
	trn._halfedges = make([]i32, max_triangles * 3, allocator)

	hash_size := int(math.ceil(math.sqrt(f32(num_coords))))
	if hash_size < 1 do hash_size = 1
	trn._hash_size = hash_size

	trn._hull_prev = make([]u32, num_coords, allocator)
	trn._hull_next = make([]u32, num_coords, allocator)
	trn._hull_tri = make([]u32, num_coords, allocator)
	trn._hull_hash = make([]i32, hash_size, allocator)
	trn._hull_buf = make([]u32, num_coords, allocator)

	trn._ids = make([]u32, num_coords, allocator)
	trn._dists = make([]f32, num_coords, allocator)
}

// update re-runs the triangulation against the current `coords` without
// reallocating any internal buffers.
delaunay_update :: proc(trn: ^Triangulation) {
	coords := trn.coords
	hull_prev := trn._hull_prev
	hull_next := trn._hull_next
	hull_tri := trn._hull_tri
	hull_hash := trn._hull_hash
	num_coords := len(coords)

	// bounding box + initialise ids
	min_x, min_y := math.inf_f32(1), math.inf_f32(1)
	max_x, max_y := math.inf_f32(-1), math.inf_f32(-1)
	for i in 0 ..< num_coords {
		x := coords[i].x
		y := coords[i].y
		if x < min_x do min_x = x
		if y < min_y do min_y = y
		if x > max_x do max_x = x
		if y > max_y do max_y = y
		trn._ids[i] = u32(i)
	}
	centre_x := (min_x + max_x) / 2
	centre_y := (min_y + max_y) / 2

	i0, i1, i2: int

	// pick a seed point close to the centre
	{
		min_dist := math.inf_f32(1)
		for i in 0 ..< num_coords {
			dd := dist(centre_x, centre_y, coords[i].x, coords[i].y)
			if dd < min_dist {
				i0 = i
				min_dist = dd
			}
		}
	}
	i0x := coords[i0].x
	i0y := coords[i0].y

	// find the point closest to the seed
	{
		min_dist := math.inf_f32(1)
		for i in 0 ..< num_coords {
			if i == i0 do continue
			dd := dist(i0x, i0y, coords[i].x, coords[i].y)
			if dd < min_dist && dd > 0 {
				i1 = i
				min_dist = dd
			}
		}
	}
	i1x := coords[i1].x
	i1y := coords[i1].y

	min_radius := math.inf_f32(1)

	// find the third point that forms the smallest circumcircle with the first two
	for i in 0 ..< num_coords {
		if i == i0 || i == i1 do continue
		radius := circumradius(i0x, i0y, i1x, i1y, coords[i].x, coords[i].y)
		if radius < min_radius {
			i2 = i
			min_radius = radius
		}
	}
	i2x := coords[i2].x
	i2y := coords[i2].y

	if min_radius == math.inf_f32(1) {
		// Collinear (or fewer than 3 distinct) points: order them by dx
		// (or dy if all x-coordinates are identical) and emit as the hull.
		for i in 0 ..< num_coords {
			dx := coords[i].x - coords[0].x
			dy := coords[i].y - coords[0].y
			trn._dists[i] = dx != 0 ? dx : dy
		}
		if num_coords > 0 do quicksort(trn._ids, trn._dists, 0, num_coords - 1)

		j := 0
		prev_dist := math.inf_f32(-1)
		for i in 0 ..< num_coords {
			id := trn._ids[i]
			dd := trn._dists[id]
			if dd > prev_dist {
				trn._hull_buf[j] = id
				j += 1
				prev_dist = dd
			}
		}
		trn.hull = trn._hull_buf[:j]
		trn.triangles = trn._triangles[:0]
		trn.halfedges = trn._halfedges[:0]
		trn.triangles_len = 0
		return
	}

	// swap the order of the seed points for counter-clockwise orientation
	if orient2d(i0x, i0y, i1x, i1y, i2x, i2y) < 0 {
		i1, i2 = i2, i1
		i1x, i2x = i2x, i1x
		i1y, i2y = i2y, i1y
	}

	circ_x, circ_y := circumcenter(i0x, i0y, i1x, i1y, i2x, i2y)
	trn._cx = circ_x
	trn._cy = circ_y

	for i in 0 ..< num_coords {
		trn._dists[i] = dist(coords[i].x, coords[i].y, circ_x, circ_y)
	}

	// sort the points by distance from the seed triangle circumcentre
	quicksort(trn._ids, trn._dists, 0, num_coords - 1)

	// set up the seed triangle as the starting hull
	trn._hull_start = u32(i0)
	hull_size := 3

	hull_next[i0] = u32(i1); hull_prev[i2] = u32(i1)
	hull_next[i1] = u32(i2); hull_prev[i0] = u32(i2)
	hull_next[i2] = u32(i0); hull_prev[i1] = u32(i0)

	hull_tri[i0] = 0
	hull_tri[i1] = 1
	hull_tri[i2] = 2

	for i in 0 ..< len(hull_hash) do hull_hash[i] = -1
	hull_hash[_hash_key(trn, i0x, i0y)] = i32(i0)
	hull_hash[_hash_key(trn, i1x, i1y)] = i32(i1)
	hull_hash[_hash_key(trn, i2x, i2y)] = i32(i2)

	trn.triangles_len = 0
	_add_triangle(trn, u32(i0), u32(i1), u32(i2), -1, -1, -1)

	prev_x, prev_y: f32
	for k in 0 ..< len(trn._ids) {
		i := int(trn._ids[k])
		x := coords[i].x
		y := coords[i].y

		// skip near-duplicate points
		if k > 0 && abs(x - prev_x) <= EPSILON && abs(y - prev_y) <= EPSILON do continue
		prev_x = x
		prev_y = y

		// skip seed triangle points
		if i == i0 || i == i1 || i == i2 do continue

		// find a visible edge on the convex hull using the edge hash
		start: u32 = 0
		{
			key := _hash_key(trn, x, y)
			for j in 0 ..< trn._hash_size {
				s := hull_hash[(key + j) % trn._hash_size]
				if s != -1 && u32(s) != hull_next[s] {
					start = u32(s)
					break
				}
			}
		}

		start = hull_prev[start]
		e := start
		q: u32
		found := true
		for {
			q = hull_next[e]
			if orient2d(x, y, coords[e].x, coords[e].y, coords[q].x, coords[q].y) < 0 do break
			e = q
			if e == start {
				found = false
				break
			}
		}
		if !found do continue // likely a near-duplicate point; skip it

		// add the first triangle from the point
		t := _add_triangle(trn, e, u32(i), hull_next[e], -1, -1, i32(hull_tri[e]))

		// recursively flip triangles from the point until the Delaunay condition is satisfied
		hull_tri[i] = u32(_legalize(trn, t + 2))
		hull_tri[e] = u32(t) // keep track of boundary triangles on the hull
		hull_size += 1

		// walk forward through the hull, adding more triangles and flipping
		next_e := hull_next[e]
		for {
			q = hull_next[next_e]
			if orient2d(x, y, coords[next_e].x, coords[next_e].y, coords[q].x, coords[q].y) >= 0 do break
			t = _add_triangle(trn, next_e, u32(i), q, i32(hull_tri[i]), -1, i32(hull_tri[next_e]))
			hull_tri[i] = u32(_legalize(trn, t + 2))
			hull_next[next_e] = next_e // mark as removed
			hull_size -= 1
			next_e = q
		}

		// walk backward from the other side, adding more triangles and flipping
		if e == start {
			for {
				q = hull_prev[e]
				if orient2d(x, y, coords[q].x, coords[q].y, coords[e].x, coords[e].y) >= 0 do break
				t = _add_triangle(trn, q, u32(i), e, -1, i32(hull_tri[e]), i32(hull_tri[q]))
				_legalize(trn, t + 2)
				hull_tri[q] = u32(t)
				hull_next[e] = e // mark as removed
				hull_size -= 1
				e = q
			}
		}

		// update the hull indices
		trn._hull_start = e
		hull_prev[i] = e
		hull_next[e] = u32(i)
		hull_prev[next_e] = u32(i)
		hull_next[i] = next_e

		// save the two new edges in the hash table
		hull_hash[_hash_key(trn, x, y)] = i32(i)
		hull_hash[_hash_key(trn, coords[e].x, coords[e].y)] = i32(e)
	}

	// build the hull
	{
		e := trn._hull_start
		for i in 0 ..< hull_size {
			trn._hull_buf[i] = e
			e = hull_next[e]
		}
	}
	trn.hull = trn._hull_buf[:hull_size]

	// trim typed triangle/halfedge arrays
	trn.triangles = trn._triangles[:trn.triangles_len]
	trn.halfedges = trn._halfedges[:trn.triangles_len]
}

// ---------------------------------------------------------------------------
// internals
// ---------------------------------------------------------------------------

// Calculate an angle-based key for the edge hash used by the advancing hull.
@(private)
_hash_key :: proc(trn: ^Triangulation, x, y: f32) -> int {
	angle := pseudo_angle(x - trn._cx, y - trn._cy)
	k := int(math.floor(angle * f32(trn._hash_size))) % trn._hash_size
	if k < 0 do k += trn._hash_size
	return k
}

// Flip an edge in a pair of triangles if it does not satisfy the Delaunay
// condition. Recursion is eliminated with a fixed-size stack.
@(private)
_legalize :: proc(trn: ^Triangulation, a_in: int) -> int {
	triangles := trn._triangles
	halfedges := trn._halfedges
	coords := trn.coords

	edge_stack: [EDGE_STACK_SIZE]u32 = ---

	a := a_in
	i := 0
	ar := 0

	for {
		b := int(halfedges[a])

		/* if the pair of triangles doesn't satisfy the Delaunay condition
		 * (p1 is inside the circumcircle of [p0, pl, pr]), flip them,
		 * then do the same check/flip recursively for the new pair of triangles
		 *
		 *           pl                    pl
		 *          /||\                  /  \
		 *       al/ || \bl            al/    \a
		 *        /  ||  \              /      \
		 *       /  a||b  \    flip    /___ar___\
		 *     p0\   ||   /p1   =>   p0\---bl---/p1
		 *        \  ||  /              \      /
		 *       ar\ || /br             b\    /br
		 *          \||/                  \  /
		 *           pr                    pr
		 */
		a0 := a - a % 3
		ar = a0 + (a + 2) % 3

		if b == -1 { 	// convex hull edge
			if i == 0 do break
			i -= 1
			a = int(edge_stack[i])
			continue
		}

		b0 := b - b % 3
		al := a0 + (a + 1) % 3
		bl := b0 + (b + 2) % 3

		p0 := triangles[ar]
		pr := triangles[a]
		pl := triangles[al]
		p1 := triangles[bl]

		illegal := in_circle(
			coords[p0].x,
			coords[p0].y,
			coords[pr].x,
			coords[pr].y,
			coords[pl].x,
			coords[pl].y,
			coords[p1].x,
			coords[p1].y,
		)

		if illegal {
			triangles[a] = p1
			triangles[b] = p0

			hbl := halfedges[bl]

			// edge swapped on the other side of the hull (rare); fix the half-edge reference
			if hbl == -1 {
				e := trn._hull_start
				for {
					if trn._hull_tri[e] == u32(bl) {
						trn._hull_tri[e] = u32(a)
						break
					}
					e = trn._hull_prev[e]
					if e == trn._hull_start do break
				}
			}
			_link(trn, a, int(hbl))
			_link(trn, b, int(halfedges[ar]))
			_link(trn, ar, bl)

			br := b0 + (b + 1) % 3

			// don't worry about hitting the cap: it can only happen on extremely degenerate input
			if i < EDGE_STACK_SIZE {
				edge_stack[i] = u32(br)
				i += 1
			}
		} else {
			if i == 0 do break
			i -= 1
			a = int(edge_stack[i])
		}
	}

	return ar
}

@(private)
_link :: proc(trn: ^Triangulation, a, b: int) {
	trn._halfedges[a] = i32(b)
	if b != -1 do trn._halfedges[b] = i32(a)
}

@(private)
_add_triangle :: proc(trn: ^Triangulation, i0, i1, i2: u32, a, b, c: i32) -> int {
	t := trn.triangles_len

	trn._triangles[t] = i0
	trn._triangles[t + 1] = i1
	trn._triangles[t + 2] = i2

	_link(trn, t, int(a))
	_link(trn, t + 1, int(b))
	_link(trn, t + 2, int(c))

	trn.triangles_len += 3
	return t
}

// ---------------------------------------------------------------------------
// geometry helpers
// ---------------------------------------------------------------------------

// Monotonically increases with the real angle, no expensive trigonometry.
@(private)
pseudo_angle :: proc(dx, dy: f32) -> f32 {
	p := dx / (abs(dx) + abs(dy))
	return (dy > 0 ? 3 - p : 1 + p) / 4 // [0..1]
}

// Squared distance between two points.
@(private)
dist :: proc(ax, ay, bx, by: f32) -> f32 {
	dx := ax - bx
	dy := ay - by
	return dx * dx + dy * dy
}

// Whether point P is inside the circle through A, B, C.
@(private)
in_circle :: proc(ax, ay, bx, by, cx, cy, px, py: f32) -> bool {
	dx := ax - px
	dy := ay - py
	ex := bx - px
	ey := by - py
	fx := cx - px
	fy := cy - py

	ap := dx * dx + dy * dy
	bp := ex * ex + ey * ey
	cp := fx * fx + fy * fy

	return dx * (ey * cp - bp * fy) - dy * (ex * cp - bp * fx) + ap * (ex * fy - ey * fx) < 0
}

// Squared circumradius of triangle ABC.
@(private)
circumradius :: proc(ax, ay, bx, by, cx, cy: f32) -> f32 {
	dx := bx - ax
	dy := by - ay
	ex := cx - ax
	ey := cy - ay

	bl := dx * dx + dy * dy
	cl := ex * ex + ey * ey
	denom := 0.5 / (dx * ey - dy * ex)

	x := (ey * bl - dy * cl) * denom
	y := (dx * cl - ex * bl) * denom
	return x * x + y * y
}

// Circumcentre of triangle ABC.
@(private)
circumcenter :: proc(ax, ay, bx, by, cx, cy: f32) -> (x, y: f32) {
	dx := bx - ax
	dy := by - ay
	ex := cx - ax
	ey := cy - ay

	bl := dx * dx + dy * dy
	cl := ex * ex + ey * ey
	denom := 0.5 / (dx * ey - dy * ex)

	x = ax + (ey * bl - dy * cl) * denom
	y = ay + (dx * cl - ex * bl) * denom
	return
}

// 2D orientation test. Positive when (a, b, c) is counter-clockwise,
// negative for clockwise, zero for collinear. NOT robust - see file header.
@(private)
orient2d :: proc(ax, ay, bx, by, cx, cy: f32) -> f32 {
	return (ax - cx) * (by - cy) - (ay - cy) * (bx - cx)
}

// ---------------------------------------------------------------------------
// quicksort by `dists[ids[i]]`
// ---------------------------------------------------------------------------

@(private)
quicksort :: proc(ids: []u32, dists: []f32, left, right: int) {
	if right - left <= 20 {
		for i := left + 1; i <= right; i += 1 {
			temp := ids[i]
			temp_dist := dists[temp]
			j := i - 1
			for j >= left && dists[ids[j]] > temp_dist {
				ids[j + 1] = ids[j]
				j -= 1
			}
			ids[j + 1] = temp
		}
	} else {
		median := (left + right) >> 1
		i := left + 1
		j := right
		ids[median], ids[i] = ids[i], ids[median]
		if dists[ids[left]] > dists[ids[right]] do ids[left], ids[right] = ids[right], ids[left]
		if dists[ids[i]] > dists[ids[right]] do ids[i], ids[right] = ids[right], ids[i]
		if dists[ids[left]] > dists[ids[i]] do ids[left], ids[i] = ids[i], ids[left]

		temp := ids[i]
		temp_dist := dists[temp]
		for {
			for {
				i += 1
				if dists[ids[i]] >= temp_dist do break
			}
			for {
				j -= 1
				if dists[ids[j]] <= temp_dist do break
			}
			if j < i do break
			ids[i], ids[j] = ids[j], ids[i]
		}
		ids[left + 1] = ids[j]
		ids[j] = temp

		if right - i + 1 >= j - left {
			quicksort(ids, dists, i, right)
			quicksort(ids, dists, left, j - 1)
		} else {
			quicksort(ids, dists, left, j - 1)
			quicksort(ids, dists, i, right)
		}
	}
}

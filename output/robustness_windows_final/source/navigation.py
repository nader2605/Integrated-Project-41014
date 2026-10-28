"""LiDAR footprint clearance and estimated-state docking. No simulator state access."""
import math
import heapq
import numpy as np
from scipy.ndimage import distance_transform_edt


def wrap(angle):
    return (angle + np.pi) % (2*np.pi) - np.pi


class Navigator:
    # Encloses the Waffle Pi including its rear chassis, plus a clearance margin.
    radius = .40

    def __init__(self, angles, track, predock=(.95, 0.), dock=(1.55, 0.)):
        self.angles = np.asarray(angles)
        self.track = track
        self.predock = np.asarray(predock)
        self.dock = np.asarray(dock)
        self.previous_direction = None
        self.docking = False
        self.reversing = False
        self.lost_frames = 0
        self.waypoint = None
        self.plan_age = 100
        self.previous_state = None
        self.search_frames = 0
        self.acquiring = False

    def plan(self, state, points):
        """A* on a short-lived LiDAR grid; unknown space is provisionally free.

        The grid contains sensor returns transformed with the EKF pose. No
        obstacle positions or robot ground truth are read from the simulator.
        """
        resolution, extent = .10, 4.
        side = int(2*extent/resolution)+1
        theta = state[2]
        rotation = np.array([[math.cos(theta), -math.sin(theta)],
                             [math.sin(theta), math.cos(theta)]])
        points = points @ rotation.T
        occupied = np.zeros((side, side), dtype=bool)
        cells = np.rint((points+extent)/resolution).astype(int)
        valid = np.all((cells >= 0) & (cells < side), axis=1)
        occupied[cells[valid, 0], cells[valid, 1]] = True
        distances = distance_transform_edt(~occupied)*resolution
        blocked = distances < self.radius+.10
        start = (side//2, side//2)
        blocked[start] = False
        goal_xy = (self.predock-state[:2]+extent)/resolution
        goal = tuple(np.clip(np.rint(goal_xy), 1, side-2).astype(int))
        if blocked[goal]:
            free = np.argwhere(~blocked)
            goal = tuple(free[np.argmin(np.sum((free-goal_xy)**2, axis=1))])
        queue = [(0., start)]
        costs, previous = {start: 0.}, {}
        moves = [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]
        while queue:
            _, cell = heapq.heappop(queue)
            if cell == goal:
                path = [cell]
                while path[-1] != start:
                    path.append(previous[path[-1]])
                path.reverse()
                distance = 0.
                chosen = path[-1]
                for a,b in zip(path, path[1:]):
                    distance += math.hypot(b[0]-a[0],b[1]-a[1])*resolution
                    line = np.rint(np.linspace(start,b,max(abs(b[0]-start[0]),abs(b[1]-start[1]))*3+2)).astype(int)
                    if np.any(blocked[line[:,0],line[:,1]]):
                        break
                    chosen = b
                    if distance >= .55:
                        break
                return state[:2]+np.asarray(chosen)*resolution-extent
            for dx,dy in moves:
                nxt = (cell[0]+dx, cell[1]+dy)
                if not (0 <= nxt[0] < side and 0 <= nxt[1] < side) or blocked[nxt]:
                    continue
                if dx and dy and (blocked[cell[0]+dx, cell[1]] or blocked[cell[0], cell[1]+dy]):
                    continue
                cost = costs[cell] + math.hypot(dx,dy)*(1+.08/(distances[nxt]+.1))
                if cost < costs.get(nxt, float('inf')):
                    costs[nxt] = cost
                    previous[nxt] = cell
                    heuristic = math.hypot(nxt[0]-goal[0],nxt[1]-goal[1])
                    heapq.heappush(queue,(cost+heuristic,nxt))
        return None

    def points(self, ranges):
        ranges = np.asarray(ranges)
        valid = np.isfinite(ranges) & (ranges > 0) & (ranges < 4.)
        return ranges[valid, None] * np.column_stack((np.cos(self.angles[valid]), np.sin(self.angles[valid])))

    def clearance(self, points, headings):
        """Centre travel before an inflated scan hit; considers the full swept disc."""
        headings = np.atleast_1d(headings)
        if not len(points):
            return np.full(len(headings), 4.)
        direction = np.column_stack((np.cos(headings), np.sin(headings)))
        along = points @ direction.T
        cross = points[:, 0, None]*direction[:, 1] - points[:, 1, None]*direction[:, 0]
        intersects = (np.abs(cross) < self.radius) & (along > 0)
        travel = along - np.sqrt(np.maximum(0, self.radius**2-cross**2))
        return np.maximum(0, np.min(np.where(intersects, travel, 4.), axis=0))

    def guard(self, points, v, w, mode):
        # Brake translation before the enclosing disc reaches an obstacle.
        heading = 0. if v >= 0 else np.pi
        distance = self.clearance(points, [heading])[0]
        limit = max(0., min(.16, .6*(distance-.08)))
        v = float(np.clip(v, -limit, limit))
        return v, float(np.clip(w, -.8, .8)), mode

    def search(self, bearing):
        # Sweep beyond the odometry heading when a supposedly centred marker
        # remains invisible. Keep turns strong enough to overcome stiction.
        self.search_frames += 1
        offset = 0.
        if self.search_frames > 200:
            offset = .35 if ((self.search_frames-200)//300) % 2 == 0 else -.35
        error = wrap(bearing+offset)
        return math.copysign(max(.55,1.4*abs(error)),error) if abs(error) > .06 else 0.

    def command(self, state, ranges, camera_accepted, landmark_bearing):
        state = np.asarray(state)
        points = self.points(ranges)
        pose_jump = self.previous_state is not None and np.linalg.norm(state[:2]-self.previous_state[:2]) > .12
        self.previous_state = state.copy()
        to_predock = self.predock-state[:2]
        self.lost_frames = 0 if camera_accepted else self.lost_frames+1
        if camera_accepted:
            self.search_frames = 0
        if not self.docking:
            distance = np.linalg.norm(to_predock)
            if distance < .20:
                self.acquiring = True
            if self.acquiring and camera_accepted and distance > .35:
                self.acquiring = False
                self.waypoint = None
            if self.acquiring:
                if camera_accepted and abs(landmark_bearing) < .18:
                    self.docking = True
                else:
                    w = self.search(landmark_bearing)
                    return self.guard(points, 0., w, 'ACQUIRING DOCK MARKER')
            else:
                self.plan_age += 1
                blocked = False
                if self.waypoint is not None:
                    delta = self.waypoint-state[:2]
                    bearing = wrap(math.atan2(delta[1],delta[0])-state[2])
                    blocked = self.clearance(points,[bearing])[0] < min(.55,np.linalg.norm(delta))+.03
                # Keep a reachable waypoint while turning towards it. Replanning
                # on every scan can alternate between opposite obstacle sides.
                if self.waypoint is None or np.linalg.norm(self.waypoint-state[:2]) < .16 or blocked or pose_jump:
                    self.waypoint = self.plan(state, points)
                    self.plan_age = 0
                if self.waypoint is None:
                    return 0., 0., 'NO SAFE LIDAR CORRIDOR'
                delta = self.waypoint-state[:2]
                heading = wrap(math.atan2(delta[1],delta[0])-state[2])
                v, w, _ = self.track(*self.waypoint)
                if abs(heading) > .22:
                    # Very small turn commands spin tyres against stiction
                    # without reliably changing base heading on this plant.
                    v, w = 0., math.copysign(max(.55, abs(1.5*heading)), heading)
                return self.guard(points, min(v, .16), w, 'LIDAR PATH + LQR')

        if self.lost_frames > 15:
            w = self.search(landmark_bearing)
            return self.guard(points, 0., w, 'REACQUIRING DOCK MARKER')

        dx = self.dock[0]-state[0]
        lateral = state[1]-self.dock[1]
        # Recover sideways error before trying to finish: reverse with the
        # camera facing the station, then approach along the docking centreline.
        if dx < .12 and abs(lateral) > .085:
            self.reversing = True
        if self.reversing and dx > .45:
            self.reversing = False
        if self.reversing:
            desired = np.clip(math.atan2(1.8*lateral, .7), -.3, .3)
            return self.guard(points, -.06, 1.8*wrap(desired-state[2]), 'DOCK REALIGNMENT')
        desired = np.clip(math.atan2(-1.8*lateral, .7), -.3, .3)*min(1.,abs(dx)/.18)
        v = np.clip(.45*dx, -.04, .10)
        w = 1.8*wrap(desired-state[2])
        if abs(wrap(desired-state[2])) > .35:
            v = 0.
        return self.guard(points, v, w, 'VISUAL STATE-FEEDBACK DOCKING')

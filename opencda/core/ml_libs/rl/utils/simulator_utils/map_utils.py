'''
Copyright 2021 OpenDILab. All Rights Reserved:
Description:
'''

import os
import math
import numpy as np
import pygame
from easydict import EasyDict
import copy

import carla
from carla import TrafficLightState as tls
from ding.utils.default_helper import deep_merge_dicts

# ==============================================================================
# -- Constants -----------------------------------------------------------------
# ==============================================================================
COLOR_BUTTER_0 = pygame.Color(252, 233, 79)
COLOR_BUTTER_1 = pygame.Color(237, 212, 0)
COLOR_BUTTER_2 = pygame.Color(196, 160, 0)

COLOR_ORANGE_0 = pygame.Color(252, 175, 62)
COLOR_ORANGE_1 = pygame.Color(245, 121, 0)
COLOR_ORANGE_2 = pygame.Color(209, 92, 0)

COLOR_CHOCOLATE_0 = pygame.Color(233, 185, 110)
COLOR_CHOCOLATE_1 = pygame.Color(193, 125, 17)
COLOR_CHOCOLATE_2 = pygame.Color(143, 89, 2)

COLOR_CHAMELEON_0 = pygame.Color(138, 226, 52)
COLOR_CHAMELEON_1 = pygame.Color(115, 210, 22)
COLOR_CHAMELEON_2 = pygame.Color(78, 154, 6)

COLOR_SKY_BLUE_0 = pygame.Color(114, 159, 207)
COLOR_SKY_BLUE_1 = pygame.Color(52, 101, 164)
COLOR_SKY_BLUE_2 = pygame.Color(32, 74, 135)

COLOR_PLUM_0 = pygame.Color(173, 127, 168)
COLOR_PLUM_1 = pygame.Color(117, 80, 123)
COLOR_PLUM_2 = pygame.Color(92, 53, 102)

COLOR_SCARLET_RED_0 = pygame.Color(239, 41, 41)
COLOR_SCARLET_RED_1 = pygame.Color(204, 0, 0)
COLOR_SCARLET_RED_2 = pygame.Color(164, 0, 0)

COLOR_ALUMINIUM_0 = pygame.Color(238, 238, 236)
COLOR_ALUMINIUM_1 = pygame.Color(211, 215, 207)
COLOR_ALUMINIUM_2 = pygame.Color(186, 189, 182)
COLOR_ALUMINIUM_3 = pygame.Color(136, 138, 133)
COLOR_ALUMINIUM_4 = pygame.Color(85, 87, 83)
COLOR_ALUMINIUM_5 = pygame.Color(46, 52, 54)

COLOR_WHITE = pygame.Color(255, 255, 255)
COLOR_BLACK = pygame.Color(0, 0, 0)

COLOR_TRAFFIC_RED = pygame.Color(255, 0, 0)
COLOR_TRAFFIC_YELLOW = pygame.Color(0, 255, 0)
COLOR_TRAFFIC_GREEN = pygame.Color(0, 0, 255)

# Module Defines
MODULE_WORLD = 'WORLD'

MAP_DEFAULT_SCALE = 0.1
HERO_DEFAULT_SCALE = 1.0

DEFAULT_BEV_CONFIG = EasyDict({
    'size': [320, 320],
    'pixels_per_meter': 5,
    'pixels_ahead_vehicle': 100,
})


# ==============================================================================
# -- Util -----------------------------------------------------------
# ==============================================================================
class Util(object):

    @staticmethod
    def blits(destination_surface, source_surfaces, rect=None, blend_mode=0):
        for surface in source_surfaces:
            destination_surface.blit(surface[0], surface[1], rect, blend_mode)

    @staticmethod
    def length(v):
        return math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)


# ==============================================================================
# -- ModuleManager -------------------------------------------------------------
# ==============================================================================
class ModuleManager(object):

    def __init__(self):
        self.modules = []

    def register_module(self, module):
        self.modules.append(module)

    def clear_modules(self):
        del self.modules[:]

    def tick(self, clock):
        # Update all the modules
        for module in self.modules:
            module.tick(clock)

    def render(self, display, snapshot=None):
        display.fill(COLOR_ALUMINIUM_4)
        for module in self.modules:
            module.render(display, snapshot=snapshot)

    def get_module(self, name):
        for module in self.modules:
            if module.name == name:
                return module

    def start_modules(self):
        for module in self.modules:
            module.start()


# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================


class MapImage(object):

    def __init__(self, carla_world, carla_map, pixels_per_meter=10):
        self._pixels_per_meter = pixels_per_meter
        self.scale = 1.0
        self._line_width = 2
        if self._pixels_per_meter < 3:
            self._line_width = 1

        waypoints = carla_map.generate_waypoints(2)
        margin = 50
        max_x = max(waypoints, key=lambda x: x.transform.location.x).transform.location.x + margin
        max_y = max(waypoints, key=lambda x: x.transform.location.y).transform.location.y + margin
        min_x = min(waypoints, key=lambda x: x.transform.location.x).transform.location.x - margin
        min_y = min(waypoints, key=lambda x: x.transform.location.y).transform.location.y - margin

        self.width = max(max_x - min_x, max_y - min_y)
        self._world_offset = (min_x, min_y)

        width_in_pixels = int(self._pixels_per_meter * self.width)

        self.big_map_surface = pygame.Surface((width_in_pixels, width_in_pixels)).convert()
        self.big_lane_surface = pygame.Surface((width_in_pixels, width_in_pixels)).convert()
        self.draw_road_map(
            self.big_map_surface, self.big_lane_surface, carla_world, carla_map, self.world_to_pixel,
            self.world_to_pixel_width
        )
        self.map_surface = self.big_map_surface
        self.lane_surface = self.big_lane_surface

    def draw_road_map(self, map_surface, lane_surface, carla_world, carla_map, world_to_pixel, world_to_pixel_width):
        # map_surface.fill(COLOR_ALUMINIUM_4)
        map_surface.fill(COLOR_BLACK)
        precision = 0.05

        def draw_lane_marking(surface, points, solid=True):
            if solid:
                # pygame.draw.lines(surface, COLOR_ORANGE_0, False, points, 2)
                pygame.draw.lines(surface, COLOR_WHITE, False, points, self._line_width)
            else:
                broken_lines = [x for n, x in enumerate(zip(*(iter(points), ) * 20)) if n % 3 == 0]
                for line in broken_lines:
                    # pygame.draw.lines(surface, COLOR_ORANGE_0, False, line, 2)
                    pygame.draw.lines(surface, COLOR_WHITE, False, line, self._line_width)

        def lateral_shift(transform, shift):
            transform.rotation.yaw += 90
            return transform.location + shift * transform.get_forward_vector()

        def does_cross_solid_line(waypoint, shift):
            w = carla_map.get_waypoint(lateral_shift(waypoint.transform, shift), project_to_road=False)
            if w is None or w.road_id != waypoint.road_id:
                return True
            else:
                return (w.lane_id * waypoint.lane_id < 0) or w.lane_id == waypoint.lane_id

        topology = [x[0] for x in carla_map.get_topology()]
        topology = sorted(topology, key=lambda w: w.transform.location.z)

        for waypoint in topology:
            waypoints = [waypoint]
            nxt = waypoint.next(precision)[0]
            while nxt.road_id == waypoint.road_id:
                waypoints.append(nxt)
                nxt = nxt.next(precision)[0]

            left_marking = [lateral_shift(w.transform, -w.lane_width * 0.5) for w in waypoints]
            right_marking = [lateral_shift(w.transform, w.lane_width * 0.5) for w in waypoints]

            polygon = left_marking + [x for x in reversed(right_marking)]
            polygon = [world_to_pixel(x) for x in polygon]

            if len(polygon) > 2:
                pygame.draw.polygon(map_surface, COLOR_WHITE, polygon, 10)
                pygame.draw.polygon(map_surface, COLOR_WHITE, polygon)

            if not waypoint.is_intersection:
                if len(left_marking) == 1:
                    continue
                sample = waypoints[int(len(waypoints) / 2)]
                draw_lane_marking(
                    lane_surface, [world_to_pixel(x) for x in left_marking],
                    does_cross_solid_line(sample, -sample.lane_width * 1.1)
                )
                draw_lane_marking(
                    lane_surface, [world_to_pixel(x) for x in right_marking],
                    does_cross_solid_line(sample, sample.lane_width * 1.1)
                )

        actors = carla_world.get_actors()
        stops_transform = [actor.get_transform() for actor in actors if 'stop' in actor.type_id]
        font_size = world_to_pixel_width(1)
        font = pygame.font.SysFont('Arial', font_size, True)
        font_surface = font.render("STOP", False, COLOR_ALUMINIUM_2)
        font_surface = pygame.transform.scale(font_surface, (font_surface.get_width(), font_surface.get_height() * 2))

    def world_to_pixel(self, location, offset=(0, 0)):
        x = self.scale * self._pixels_per_meter * \
            (location.x - self._world_offset[0])
        y = self.scale * self._pixels_per_meter * \
            (location.y - self._world_offset[1])
        return [int(x - offset[0]), int(y - offset[1])]

    def world_to_pixel_width(self, width):
        return int(self.scale * self._pixels_per_meter * width)

    def scale_map(self, scale):
        if scale != self.scale:
            self.scale = scale
            width = int(self.big_map_surface.get_width() * self.scale)
            self.surface = pygame.transform.smoothscale(self.big_map_surface, (width, width))


class ModuleWorld(object):

    def __init__(
        self, name, client, world, town_map, hero_actor, width, height, pixels_per_meter, pixels_ahead_vehicle
    ):

        self.name = name
        self.server_fps = 0.0
        self.simulation_time = 0

        self.server_clock = pygame.time.Clock()

        self.window_width = width
        self.window_height = height
        self.pixels_per_meter = pixels_per_meter
        self.pixel_ahead_vehicle = pixels_ahead_vehicle

        # World data
        self.client = client
        self.world = world
        self.town_map = town_map
        self.actors_with_transforms = []
        self.hero_waypoints = None

        self.surface_size = [0, 0]
        self.prev_scaled_size = 0
        self.scaled_size = 0
        # Hero actor
        self.hero_actor = hero_actor
        self.hero_transform = hero_actor.get_transform()

        self.scale_offset = [0, 0]

        self.traffic_light_surfaces = None
        self.affected_traffic_light = None

        # Map info
        self.map_image = None
        self.original_surface_size = None

        self.self_surface = None
        self.vehicle_surface = None
        self.walker_surface = None
        self.waypoint_surface = None

        self.hero_map_surface = None
        self.hero_lane_surface = None
        self.hero_self_surface = None
        self.hero_vehicle_surface = None
        self.hero_walker_surface = None
        self.hero_traffic_light_surface = None
        self.hero_waypoint_surface = None

        self.window_map_surface = None
        self.window_lane_surface = None
        self.window_self_surface = None
        self.window_vehicle_surface = None
        self.window_walker_surface = None
        self.window_traffic_light_surface = None
        self.window_waypoint_surface = None

        self.hero_map_image = None
        self.hero_lane_image = None
        self.hero_self_image = None
        self.hero_vehicle_image = None
        self.hero_walker_image = None
        self.hero_traffic_image = None
        self.hero_waypoint_image = None

    def get_rendered_surfaces(self):
        return (
            self.hero_map_image,
            self.hero_lane_image,
            self.hero_self_image,
            self.hero_vehicle_image,
            self.hero_walker_image,
            self.hero_traffic_image,
            self.hero_waypoint_image,
        )

    def start(self):
        # Create Surfaces
        self.map_image = MapImage(self.world, self.town_map, self.pixels_per_meter)

        self.original_surface_size = min(self.window_height, self.window_height)
        self.surface_size = self.map_image.big_map_surface.get_width()

        self.scaled_size = int(self.surface_size)
        self.prev_scaled_size = int(self.surface_size)

        # Render Actors
        self.vehicle_surface = pygame.Surface(
            (self.map_image.map_surface.get_width(), self.map_image.map_surface.get_height())
        )
        self.vehicle_surface.set_colorkey(COLOR_BLACK)
        self.self_surface = pygame.Surface(
            (self.map_image.map_surface.get_width(), self.map_image.map_surface.get_height())
        )
        self.self_surface.set_colorkey(COLOR_BLACK)
        self.walker_surface = pygame.Surface(
            (self.map_image.map_surface.get_width(), self.map_image.map_surface.get_height())
        )
        self.walker_surface.set_colorkey(COLOR_BLACK)
        self.traffic_light_surface = pygame.Surface(
            (self.map_image.map_surface.get_width(), self.map_image.map_surface.get_height())
        )
        self.traffic_light_surface.set_colorkey(COLOR_BLACK)
        self.waypoint_surface = pygame.Surface(
            (self.map_image.map_surface.get_width(), self.map_image.map_surface.get_height())
        )
        self.waypoint_surface.set_colorkey(COLOR_BLACK)

        scaled_original_size = self.original_surface_size * (1.0 / 0.9)

        self.hero_map_surface = pygame.Surface((scaled_original_size, scaled_original_size)).convert()
        self.hero_lane_surface = pygame.Surface((scaled_original_size, scaled_original_size)).convert()
        self.hero_self_surface = pygame.Surface((scaled_original_size, scaled_original_size)).convert()
        self.hero_vehicle_surface = pygame.Surface((scaled_original_size, scaled_original_size)).convert()
        self.hero_walker_surface = pygame.Surface((scaled_original_size, scaled_original_size)).convert()
        self.hero_traffic_light_surface = pygame.Surface((scaled_original_size, scaled_original_size)).convert()
        self.hero_waypoint_surface = pygame.Surface((scaled_original_size, scaled_original_size)).convert()

        self.window_map_surface = pygame.Surface((self.window_width, self.window_height)).convert()
        self.window_lane_surface = pygame.Surface((self.window_width, self.window_height)).convert()
        self.window_self_surface = pygame.Surface((self.window_width, self.window_height)).convert()
        self.window_vehicle_surface = pygame.Surface((self.window_width, self.window_height)).convert()
        self.window_walker_surface = pygame.Surface((self.window_width, self.window_height)).convert()
        self.window_traffic_light_surface = pygame.Surface((self.window_width, self.window_height)).convert()
        self.window_waypoint_surface = pygame.Surface((self.window_width, self.window_height)).convert()

        self.wheel_offset = HERO_DEFAULT_SCALE

    def tick(self):
        actors = self.world.get_actors()
        self.actors_with_transforms = [(actor, actor.get_transform()) for actor in actors]
        if self.hero_actor is not None:
            self.hero_transform = self.hero_actor.get_transform()

    def _split_actors(self):
        vehicles = []
        traffic_lights = []
        walkers = []

        for actor_with_transform in self.actors_with_transforms:
            actor = actor_with_transform[0]
            if 'vehicle' in actor.type_id:
                vehicles.append(actor_with_transform)
            elif 'traffic_light' in actor.type_id:
                traffic_lights.append(actor_with_transform)
            elif 'walker' in actor.type_id:
                walkers.append(actor_with_transform)

        return (vehicles, traffic_lights, walkers)

    def get_bounding_box(self, actor):
        bb = actor.trigger_volume.extent
        corners = [
            carla.Location(x=-bb.x, y=-bb.y),
            carla.Location(x=bb.x, y=-bb.y),
            carla.Location(x=bb.x, y=bb.y),
            carla.Location(x=-bb.x, y=bb.y),
            carla.Location(x=-bb.x, y=-bb.y)
        ]
        corners = [x + actor.trigger_volume.location for x in corners]
        t = actor.get_transform()
        t.transform(corners)
        return corners

    def _render_traffic_lights(self, surface, list_tl, world_to_pixel, world_to_pixel_width, from_snapshot=False):
        self.affected_traffic_light = None

        for tl in list_tl:
            if from_snapshot:
                world_pos = carla.Location(
                    x=tl["location"]["x"],
                    y=tl["location"]["y"],
                )
            else:
                world_pos = tl.get_location()
            pos = world_to_pixel(world_pos)

            if from_snapshot:
                if tl["state"] == int(tls.Red):
                    # color = COLOR_SCARLET_RED_0
                    color = COLOR_TRAFFIC_RED
                elif tl["state"] == int(tls.Yellow):
                    color = COLOR_TRAFFIC_YELLOW
                    # color = COLOR_BUTTER_0
                elif tl["state"] == int(tls.Green):
                    color = COLOR_TRAFFIC_GREEN
                    # color = COLOR_CHAMELEON_0
                else:
                    continue  # Unknown or off traffic light
            else:
                if tl.state == tls.Red:
                    # color = COLOR_SCARLET_RED_0
                    color = COLOR_TRAFFIC_RED
                elif tl.state == tls.Yellow:
                    color = COLOR_TRAFFIC_YELLOW
                    # color = COLOR_BUTTER_0
                elif tl.state == tls.Green:
                    color = COLOR_TRAFFIC_GREEN
                    # color = COLOR_CHAMELEON_0
                else:
                    continue  # Unknown or off traffic light

            # Draw circle instead of rectangle
            radius = world_to_pixel_width(1.5)
            pygame.draw.circle(surface, color, pos, radius)

    def _render_walkers(self, surface, list_w, world_to_pixel, from_snapshot=False):
        # print ("Walkers")

        for w in list_w:
            color = COLOR_WHITE
            # Compute bounding box points
            if from_snapshot:
                bb = w["bbox"]
                corners = [
                    carla.Location(x=-bb["x"], y=-bb["y"]),
                    carla.Location(x=bb["x"], y=-bb["y"]),
                    carla.Location(x=bb["x"], y=bb["y"]),
                    carla.Location(x=-bb["x"], y=bb["y"])
                ]
                w_location = carla.Location(x=w["location"]["x"], y=w["location"]["y"])
                corners = [corner + w_location for corner in corners]
            else:
                if not hasattr(w[0], 'bounding_box'):
                    continue

                bb = w[0].bounding_box.extent
                corners = [
                    carla.Location(x=-bb.x, y=-bb.y),
                    carla.Location(x=bb.x, y=-bb.y),
                    carla.Location(x=bb.x, y=bb.y),
                    carla.Location(x=-bb.x, y=bb.y)
                ]
                w[1].transform(corners)

            corners = [world_to_pixel(p) for p in corners]
            # print (corners)
            pygame.draw.polygon(surface, color, corners)

    def _render_vehicles(self, vehicle_surface, self_surface, list_v, world_to_pixel, from_snapshot=False):
        # print ("rendered a car?!")
        for v in list_v:
            # color = COLOR_SKY_BLUE_0
            color = COLOR_WHITE

            if not from_snapshot and v[0].attributes['role_name'] == 'hero':
                # Do not render othre vehicles
                # print (v[1])
                surface = self_surface
            else:
                surface = vehicle_surface

                # continue # Do not render itself
            # Compute bounding box points
            if from_snapshot:
                bb = v["bbox"]
                corners = [
                    carla.Location(x=-bb["x"], y=-bb["y"]),
                    carla.Location(x=bb["x"], y=-bb["y"]),
                    carla.Location(x=bb["x"], y=bb["y"]),
                    carla.Location(x=-bb["x"], y=bb["y"])
                ]
                v_location = carla.Location(x=v["location"]["x"], y=v["location"]["y"])
                corners = [corner + v_location for corner in corners]
            else:
                bb = v[0].bounding_box.extent
                corners = [
                    carla.Location(x=-bb.x, y=-bb.y),
                    carla.Location(x=-bb.x, y=bb.y),
                    carla.Location(x=bb.x, y=bb.y),
                    carla.Location(x=bb.x, y=-bb.y)
                ]
                v[1].transform(corners)
            # print ("Vehicle")
            corners = [world_to_pixel(p) for p in corners]
            pygame.draw.polygon(surface, color, corners)

    def _render_waypoints(self, waypoint_surface, waypoint_list, world_to_pixel, world_to_pixel_width):
        prev_x = None
        prev_y = None
        # wp_location = waypoint_list[0].transform.location
        # x, y = world_to_pixel(wp_location)
        # pygame.draw.circle(waypoint_surface, pygame.Color(0, 255, 0), (x, y), 4)
        for wp in waypoint_list[1:]:
            wp_location = wp.transform.location
            lane_width = world_to_pixel_width(wp.lane_width * 0.5)
            x, y = world_to_pixel(wp_location)
            if prev_x is not None:
                pygame.draw.line(waypoint_surface, (0, 255, 0), (x, y), (prev_x, prev_y), lane_width)
            prev_x = x
            prev_y = y
            # radius = 2
            # x, y = world_to_pixel(wp_location)
            # pygame.draw.circle(waypoint_surface, pygame.Color(0, 100, 0), (x, y), radius)

    def render_actors(
        self,
        vehicle_surface,
        self_surface,
        walker_surface,
        traffic_light_surface,
        vehicles,
        traffic_lights,
        walkers,
        from_snapshot=False
    ):
        # Static actors

        # TODO: render traffic lights and speed limits on respective channels
        if from_snapshot:
            self._render_traffic_lights(
                traffic_light_surface,
                traffic_lights,
                self.map_image.world_to_pixel,
                self.map_image.world_to_pixel_width,
                from_snapshot=True
            )
        else:
            self._render_traffic_lights(
                traffic_light_surface, [tl[0] for tl in traffic_lights],
                self.map_image.world_to_pixel,
                self.map_image.world_to_pixel_width,
                from_snapshot=False
            )

        # Dynamic actors
        self._render_vehicles(
            vehicle_surface, self_surface, vehicles, self.map_image.world_to_pixel, from_snapshot=from_snapshot
        )
        self._render_walkers(walker_surface, walkers, self.map_image.world_to_pixel, from_snapshot=from_snapshot)

    def clip_surfaces(self, clipping_rect):
        self.vehicle_surface.set_clip(clipping_rect)
        self.walker_surface.set_clip(clipping_rect)
        self.traffic_light_surface.set_clip(clipping_rect)

    def render(self, display, snapshot=None):
        if snapshot is None and self.actors_with_transforms is None:
            return

        if snapshot is None:
            vehicles, traffic_lights, walkers = self._split_actors()
        else:
            vehicles = snapshot["vehicles"]
            traffic_lights = snapshot["traffic_lights"]
            walkers = snapshot["walkers"]

        scale_factor = self.wheel_offset
        self.scaled_size = int(self.map_image.width * scale_factor)

        # Render Actors
        self.vehicle_surface.fill(COLOR_BLACK)
        self.walker_surface.fill(COLOR_BLACK)
        self.traffic_light_surface.fill(COLOR_BLACK)
        self.self_surface.fill(COLOR_BLACK)
        self.waypoint_surface.fill(COLOR_BLACK)

        self.render_actors(
            self.vehicle_surface,
            self.self_surface,
            self.walker_surface,
            self.traffic_light_surface,
            vehicles,
            traffic_lights,
            walkers,
            from_snapshot=(snapshot is not None)
        )

        if self.hero_waypoints is not None:
            self._render_waypoints(
                self.waypoint_surface, self.hero_waypoints, self.map_image.world_to_pixel,
                self.map_image.world_to_pixel_width
            )

        center_offset = (0, 0)
        angle = 0.0 if self.hero_actor is None else self.hero_transform.rotation.yaw + 90

        if self.hero_actor is not None:
            if snapshot is None:
                hero_front = self.hero_transform.get_forward_vector()
                hero_location_screen = self.map_image.world_to_pixel(self.hero_transform.location)
            else:
                hero_location = snapshot["player"]["transform"]["location"]
                hero_location = carla.Location(
                    x=hero_location["x"],
                    y=hero_location["y"],
                    z=hero_location["z"],
                )
                hero_location_screen = self.map_image.world_to_pixel(hero_location)

                hero_orientation = snapshot["player"]["transform"]["orientation"]
                hero_front = carla.Location(x=hero_orientation["x"], y=hero_orientation["y"])

            offset = [0, 0]
            offset[0] += hero_location_screen[0] - \
                self.hero_map_surface.get_width() / 2
            offset[0] += hero_front.x * self.pixel_ahead_vehicle
            offset[1] += hero_location_screen[1] - \
                self.hero_map_surface.get_height() / 2
            offset[1] += hero_front.y * self.pixel_ahead_vehicle

            # Apply clipping rect
            clipping_rect = pygame.Rect(
                offset[0], offset[1], self.hero_map_surface.get_width(), self.hero_map_surface.get_height()
            )

            self.clip_surfaces(clipping_rect)

            self.hero_self_surface.fill(COLOR_BLACK)
            self.hero_map_surface.fill(COLOR_BLACK)
            self.hero_vehicle_surface.fill(COLOR_BLACK)
            self.hero_walker_surface.fill(COLOR_BLACK)
            self.hero_traffic_light_surface.fill(COLOR_BLACK)
            self.hero_waypoint_surface.fill(COLOR_BLACK)

            self.hero_self_surface.blit(self.self_surface, (-offset[0], -offset[1]))
            self.hero_map_surface.blit(self.map_image.map_surface, (-offset[0], -offset[1]))
            self.hero_lane_surface.blit(self.map_image.lane_surface, (-offset[0], -offset[1]))
            self.hero_vehicle_surface.blit(self.vehicle_surface, (-offset[0], -offset[1]))
            self.hero_walker_surface.blit(self.walker_surface, (-offset[0], -offset[1]))
            self.hero_traffic_light_surface.blit(self.traffic_light_surface, (-offset[0], -offset[1]))
            self.hero_waypoint_surface.blit(self.waypoint_surface, (-offset[0], -offset[1]))

            # Rotate: map/vehicle/walker surface
            rz = pygame.transform.rotozoom

            rotated_map_surface = rz(self.hero_map_surface, angle, 0.9).convert()
            rotated_lane_surface = rz(self.hero_lane_surface, angle, 0.9).convert()
            rotated_vehicle_surface = rz(self.hero_vehicle_surface, angle, 0.9).convert()
            rotated_walker_surface = rz(self.hero_walker_surface, angle, 0.9).convert()
            rotated_traffic_surface = rz(self.hero_traffic_light_surface, angle, 0.9).convert()
            rotated_self_surface = rz(self.hero_self_surface, angle, 0.9).convert()
            rotated_waypoint_surface = rz(self.hero_waypoint_surface, angle, 0.9).convert()

            center = (display.get_width() / 2, display.get_height() / 2)
            rotation_map_pivot = rotated_map_surface.get_rect(center=center)
            rotation_lane_pivot = rotated_lane_surface.get_rect(center=center)
            rotation_vehicle_pivot = rotated_vehicle_surface.get_rect(center=center)
            rotation_walker_pivot = rotated_walker_surface.get_rect(center=center)
            rotation_traffic_pivot = rotated_traffic_surface.get_rect(center=center)
            rotation_self_pivot = rotated_self_surface.get_rect(center=center)
            rotation_waypoint_pivot = rotated_waypoint_surface.get_rect(center=center)

            self.window_map_surface.blit(rotated_map_surface, rotation_map_pivot)
            self.window_lane_surface.blit(rotated_lane_surface, rotation_lane_pivot)
            self.window_vehicle_surface.blit(rotated_vehicle_surface, rotation_vehicle_pivot)
            self.window_walker_surface.blit(rotated_walker_surface, rotation_walker_pivot)
            self.window_traffic_light_surface.blit(rotated_traffic_surface, rotation_traffic_pivot)
            self.window_self_surface.blit(rotated_self_surface, rotation_self_pivot)
            self.window_waypoint_surface.blit(rotated_waypoint_surface, rotation_waypoint_pivot)

            def make_image(x):
                return np.swapaxes(pygame.surfarray.array3d(x), 0, 1).mean(axis=-1)

            # Save surface as rgb array
            self.hero_map_image = make_image(self.window_map_surface)
            self.hero_lane_image = make_image(self.window_lane_surface)
            self.hero_vehicle_image = make_image(self.window_vehicle_surface)
            self.hero_walker_image = make_image(self.window_walker_surface)
            self.hero_traffic_image = np.swapaxes(pygame.surfarray.array3d(self.window_traffic_light_surface), 0, 1)
            self.hero_self_image = make_image(self.window_self_surface)
            self.hero_waypoint_image = make_image(self.window_waypoint_surface)


# ==============================================================================
# bradyz: Wrap all this --------------------------------------------------------
# ==============================================================================


class BeVWrapper(object):

    config = dict(
        size=[320, 320],
        pixels_per_meter=5,
        pixels_ahead_vehicle=100,
    )

    def __init__(self, cfg):
        """
        docstring
        """
        if 'cfg_type' not in cfg:
            self._cfg = self.__class__.default_config()
            self._cfg = deep_merge_dicts(self._cfg, cfg)
        else:
            self._cfg = cfg
        self.clock = None
        self.display = None
        self.world_module = None
        self.width = self._cfg.size[0]
        self.height = self._cfg.size[1]
        self.pixels_per_meter = self._cfg.pixels_per_meter
        self.pixels_ahead_vehicle = self._cfg.pixels_ahead_vehicle

    def init(self, client, world, carla_map, player, route=None):
        os.environ['SDL_VIDEODRIVER'] = 'dummy'

        pygame.init()
        self.display = pygame.display.set_mode((self.width, self.height), 0, 32)
        pygame.display.flip()

        # Set map drawer module
        self.world_module = ModuleWorld(
            MODULE_WORLD, client, world, carla_map, player, self.width, self.height, self.pixels_per_meter,
            self.pixels_ahead_vehicle
        )

        self.world_module.start()

    def tick(self):
        self.world_module.tick()
        self.display.fill(COLOR_ALUMINIUM_4)
        self.world_module.render(self.display)

    def update_waypoints(self, waypoints):
        self.world_module.hero_waypoints = waypoints

    def get_bev_data(self):
        road, lane, hero, vehicle, pedestrian, traffic, route = self.world_module.get_rendered_surfaces()

        result = {
            'hero': np.uint8(hero),
            'road': np.uint8(road),
            'lane': np.uint8(lane),
            'vehicle': np.uint8(vehicle),
            'pedestrian': np.uint8(pedestrian),
            'traffic': np.uint8(traffic),
            'route': np.uint8(route),
        }

        return result

    def clear(self):
        del self.world_module
        self.world_module = None

    def world_to_pixel(self, pos):
        return self.world_module.map_image.world_to_pixel(pos)

    @classmethod
    def default_config(cls: type) -> EasyDict:
        cfg = EasyDict(cls.config)
        cfg.cfg_type = cls.__name__ + 'Config'
        return copy.deepcopy(cfg)

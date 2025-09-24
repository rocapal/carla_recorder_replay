#!/usr/bin/env python3
#
#
#  Copyright (C) Roberto Calvo Palomino
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see http://www.gnu.org/licenses/. 
#
#  Author : Roberto Calvo Palomino <roberto.calvo at urjc dot es
#


import carla
import pygame
import numpy as np
import sys
import time

log_filename = "/tmp/town04_autopilot.log"


def get_log_duration(client, log_file):
    import re            
    info = client.show_recorder_file_info(log_file, False)  
    # Look at for Duration: 12.34 s"
    match = re.search(r"Duration:\s+([0-9.]+)", info)
    if not match:
        raise RuntimeError("No se pudo leer duración del log")
    return float(match.group(1))

def replay_loop(view="car"):
    pygame.init()
    pygame.display.set_caption(f"CARLA Replay - Replay view {view} ")
    display_width, display_height = 800, 600
    screen = pygame.display.set_mode((display_width, display_height))

    client = carla.Client('localhost', 3010)  
    client.set_timeout(10.0)

    world = client.get_world()

    # Synchronous mode     
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)


    duration = get_log_duration(client, log_filename)
    duration = duration + world.get_snapshot().timestamp.elapsed_seconds
    print(f"Replaying: {log_filename}, duration: {duration:.2f} s")    
    

    client.replay_file(log_filename, 0, 0, 0)


    blueprint_library = world.get_blueprint_library()

    # Required to make a tick for actors to appear in the scene
    world.tick()    
    
    actors = None

    if (view == "car"):
        actors = world.get_actors().filter("vehicle.tesla.model3")
    elif (view == "bike"):
        actors = world.get_actors().filter("vehicle.diamondback.century")

    if not actors:
        raise RuntimeError("No vehicles found in the replay")

    vehicle = actors[0]  # suponemos que el primer vehículo es el ego
    print(f"Using ego con id={vehicle.id}, type={vehicle.type_id}")
    
    
    
    camera_bp = blueprint_library.find("sensor.camera.rgb")
    camera_bp.set_attribute("image_size_x", str(display_width))
    camera_bp.set_attribute("image_size_y", str(display_height))
    camera_bp.set_attribute("fov", "90")

    camera_transform = carla.Transform(carla.Location(x=0.8, z=1.7))
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)

    image_surface = None

    def process_image(image):
        nonlocal image_surface
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        image_surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

    camera.listen(lambda img: process_image(img))

    clock = pygame.time.Clock()

    try:
        while True:            

            world.tick()

            snapshot = world.get_snapshot()                        
            
            if snapshot.timestamp.elapsed_seconds >= duration:
                print("Replay finished")
                break

            # You can get the controls of the vehicule at each snapshot
            # ctrl = vehicle.get_control()
            # print(ctrl.throttle, ctrl.steer, ctrl.brake)

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    raise KeyboardInterrupt

            if image_surface is not None:
                screen.blit(image_surface, (0, 0))

            pygame.display.flip()

    except KeyboardInterrupt:
        print("Exit...")

    finally:
        if camera is not None:
            camera.stop()
            camera.destroy()

        vehicle.destroy()

        settings.synchronous_mode = False
        world.apply_settings(settings)


        pygame.quit()
        sys.exit()


if __name__ == "__main__":

    # Use "bike" or "car" to choose from where point of view you want to replay de simulation
    replay_loop("car")

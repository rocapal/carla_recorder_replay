#!/usr/bin/env python3
#
#
#  Copyright (C) URJC DeepRacer
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
#           Sergio Robledo <s.robledo.2021 at alumnos dot urjc dot es>


import carla
import pygame
import numpy as np
import sys
import time
import os
import argparse
import csv
import cv2
from pathlib import Path

import queue
from queue import Queue

from dataset_manager import DatasetSaver

RATE_CONTROL_LOOP = 30

import pandas as pd

class DatasetSaverMultiAgent(DatasetSaver):
    def __init__ (self, log_path):
        # Determine the output path: dataset, dataset-1, dataset-2...
        base = Path(log_path)
        name = "dataset"
        final_path = base / name
        
        count = 1
        while final_path.exists():
            final_path = base / f"{name}-{count}"
            count += 1
            
        self.dataset_path = str(final_path) + "/"
        self.path = log_path

        self.rgb_foldername = "rgb"
        self.mask_foldername = "mask"

        self.rgb_path = os.path.join(self.dataset_path, self.rgb_foldername)
        self.mask_path = os.path.join(self.dataset_path, self.mask_foldername)
        self.csv_filename = os.path.join(self.dataset_path, "dataset.csv")

        self.counter = 0

        os.makedirs(self.rgb_path, exist_ok=True)
        os.makedirs(self.mask_path, exist_ok=True)

        print (f"DatasetSaver loaded for {self.dataset_path}")
        
        # Initialize CSV
        with open(self.csv_filename, "w", newline="") as f:
            csv.writer(f).writerow(["rgb_path","mask_path","timestamp",
                                    "ego_throttle","ego_steer","ego_brake",
                                    "ego_speed","vru_speed",
                                    "ego_x","ego_y","ego_yaw","vru_x","vru_y","vru_yaw"])

    def save_sample (self, timestamp, bgr, mask_rgb, ego_throttle, ego_steer, ego_brake, ego_speed, vru_speed, ego_x, ego_y, ego_yaw, vru_x, vru_y, vru_yaw):
        rgb_filename  = f"rgb_{self.counter:08d}.png"
        mask_filename = f"mask_{self.counter:08d}.png"
        
        self.counter = self.counter + 1

        cv2.imwrite(os.path.join(self.rgb_path,  rgb_filename),  bgr)
        cv2.imwrite(os.path.join(self.mask_path, mask_filename),
                    cv2.cvtColor(mask_rgb, cv2.COLOR_RGB2BGR))

        with open(self.csv_filename, "a", newline="") as f:
            csv.writer(f).writerow([
                f"/{self.rgb_foldername}/{rgb_filename}",
                f"/{self.mask_foldername}/{mask_filename}",
                timestamp, ego_throttle, ego_steer, ego_brake, 
                ego_speed, vru_speed,
                ego_x, ego_y, ego_yaw, vru_x, vru_y, vru_yaw
            ]) 

    def adjust_speed (self, csv_data_filename):
        try:
            self.load_speed_from_csv(
                self.csv_filename,
                csv_data_filename,
                col_mapping={
                    "ego_speed": "ego_speed_kmh",
                    "vru_speed": "vru_speed_kmh"
                }
            )
        except Exception as e:
            print(f"[ERROR] speed align (merge_asof): {e}")

    def load_speed_from_csv(self,
                            dataset_csv: str,
                            speed_csv: str,
                            col_mapping: dict,
                            src_time_col: str = "sim_time"):

        # 1) Check files
        if not os.path.isfile(dataset_csv):
            print(f"[ERROR] Dataset {dataset_csv} does not exist")
            return
        if not os.path.isfile(speed_csv):
            print(f"[ERROR] Speed CSV does not exist: {speed_csv}")
            return

        # 2) Loading
        df_dst = pd.read_csv(dataset_csv)
        df_src = pd.read_csv(speed_csv)

        # 3) Check necessary columns
        if "timestamp" not in df_dst.columns:
            print(f"[ERROR] Dataset does not have col 'timestamp'.")
            return

        for dst_col in col_mapping.keys():
             if dst_col not in df_dst.columns:
                print(f"[ERROR] Dataset does not have col '{dst_col}'.")
                return

        if src_time_col not in df_src.columns:
            print(f"[ERROR] Speed CSV not containing col '{src_time_col}'.")
            return

        for src_col in col_mapping.values():
            if src_col not in df_src.columns:
                print(f"[ERROR] Speed CSV not containing col '{src_col}'.")
                return

        if df_dst.empty or df_src.empty:
            print("[WARN] Empty Dataset or speed CSV")
            return

        # 4) Number conversion
        df_dst = df_dst.copy()
        df_src = df_src.copy()

        df_dst["timestamp"]      = pd.to_numeric(df_dst["timestamp"], errors="coerce")
        df_src[src_time_col]     = pd.to_numeric(df_src[src_time_col], errors="coerce")
        
        for src_col in col_mapping.values():
            df_src[src_col] = pd.to_numeric(df_src[src_col], errors="coerce")

        df_dst = df_dst.dropna(subset=["timestamp"])
        df_src = df_src.dropna(subset=[src_time_col])

        if df_dst.empty or df_src.empty:
            print("[WARN] NaN data filtered and no data left.")
            return

        # 5) Order by time
        df_dst_sorted = df_dst.sort_values("timestamp").reset_index(drop=False)
        df_src_sorted = df_src.sort_values(src_time_col).reset_index(drop=True)

        # 6) Align using merge_asof
        src_cols_to_use = [src_time_col] + list(col_mapping.values())
        
        merged = pd.merge_asof(
            df_dst_sorted,
            df_src_sorted[src_cols_to_use],
            left_on="timestamp",
            right_on=src_time_col,
            direction="nearest"
        )

        # 7) Set aligned speed in the original DataFrame
        for dst_col, src_col in col_mapping.items():
            df_dst.loc[merged["index"], dst_col] = merged[src_col].values

        # 8) Keep updated data
        df_dst.to_csv(dataset_csv, index=False)

        # 9) Info
        time_diff = np.abs(merged["timestamp"] - merged[src_time_col])
        print(f"[INFO] Speed alignment")
        print(f"  - Nº rows dataset:   {len(df_dst)}")
        print(f"  - Nº rows speed_csv: {len(df_src)}")
        print(f"  - Avg time diff:     {time_diff.mean():.4f} s")
        print(f"  - Max time diff:     {time_diff.max():.4f} s")


def get_log_duration(client, log_file):
    import re           
    info = client.show_recorder_file_info(log_file, False)  
    # Look at for Duration: 12.34 s"
    match = re.search(r"Duration:\s+([0-9.]+)", info)
    if not match:
        raise RuntimeError("Duration time cannot be read!")
    return float(match.group(1))

def replay_loop(args, view="car"):

    pygame.init()
    pygame.display.set_caption(f"CARLA Replay - Replay view {view} ")
    display_width, display_height = 800, 600
    screen = pygame.display.set_mode((display_width, display_height))

    client = carla.Client('localhost', args.port)  
    client.set_timeout(10.0)

    world = client.get_world()
    
    path = Path(args.log_path)
    logs = list(path.glob("*.log"))    
    if (len(logs) == 0):
        print(f"Error, no log file found in {args.log_path}")
        exit(-1)    

    p = logs[0]
    if not p.is_absolute():
        p = Path.cwd() / p
    log_filename = str(p)
    print(f'Using log file {log_filename}')

    duration = get_log_duration(client, log_filename)
    duration = duration + world.get_snapshot().timestamp.elapsed_seconds
    print(f"Replaying: {log_filename}, duration: {duration:.2f} s")

    # Always generate dataset in the log path
    dataset = DatasetSaverMultiAgent(args.log_path)
    
    # Initialize actors to None for safe cleanup
    camera = None
    vehicle = None

    try:
        client.replay_file(log_filename, 0, 0, 0)
        time.sleep(0.1)
        world.wait_for_tick()

        blueprint_library = world.get_blueprint_library()

        # Get all actors
        actors = world.get_actors().filter("vehicle.*")    
        
        ego_actor = None
        vru_actor = None

        for actor in actors:
            if "model3" in actor.type_id:
                ego_actor = actor
            elif "century" in actor.type_id:
                vru_actor = actor

        if (view == "car"):
            if ego_actor is not None:
                vehicle = ego_actor
            else:
                 print("Warning: Ego actor (model3) not found. Using first actor found.")
                 if len(actors) > 0:
                    vehicle = actors[0]
                 else:
                    raise RuntimeError("No vehicles found in the replay")

        elif (view == "bike"):
            if vru_actor is not None:
                vehicle = vru_actor
            else:
                 print("Warning: VRU actor (century) not found. Using first actor found.")
                 if len(actors) > 0:
                    vehicle = actors[0]
                 else:
                    raise RuntimeError("No vehicles found in the replay")

        if not vehicle:
            raise RuntimeError("No vehicles found in the replay")

        print(f"Using vehicle id={vehicle.id}, type={vehicle.type_id}")
        
        camera_bp = blueprint_library.find("sensor.camera.rgb")
        camera_bp.set_attribute("image_size_x", str(display_width))
        camera_bp.set_attribute("image_size_y", str(display_height))
        camera_bp.set_attribute("fov", "90")

        camera_transform = carla.Transform(carla.Location(x=0.8, z=1.7))
        camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)

        frame_q = Queue(maxsize=1)   # save (rgb, bgr, (w, h))

        def _safe_put(q: Queue, item):
            try:
                q.put_nowait(item)
            except queue.Full:
                try:
                    q.get_nowait()
                except queue.Empty:
                    pass
                q.put_nowait(item)

        def process_image(image):
            bgra = np.frombuffer(image.raw_data, dtype=np.uint8)
            bgra = np.reshape(bgra, (image.height, image.width, 4))
            bgr  = bgra[:, :, :3].copy()
            rgb  = bgr[:, :, ::-1]
            _safe_put(frame_q, (rgb, bgr, (image.width, image.height)))

        camera.listen(lambda img: process_image(img))

        clock = pygame.time.Clock()

        # Start at a relative time 0.0 to syncronize with speed csv
        t0_sim = 0.0

        while True:            
            
            clock.tick(RATE_CONTROL_LOOP)

            snapshot = world.get_snapshot()  
            sim_time = snapshot.timestamp.elapsed_seconds                      
            
            if sim_time >= duration:
                print("Replay finished")
                break

            try:
                rgb, bgr, (w, h) = frame_q.get_nowait()
            except queue.Empty:
             
                for e in pygame.event.get():
                    if e.type == pygame.QUIT:
                        raise KeyboardInterrupt
                continue

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    raise KeyboardInterrupt

            # Get relative time for speedcsv/replay sync
            if t0_sim == 0.0:
                t0_sim = sim_time

            rel_time = sim_time - t0_sim

            
            #if image_surface is not None:
            surface = pygame.surfarray.make_surface(rgb.swapaxes(0, 1))
            screen.blit(surface, (0, 0))

            pygame.display.flip()

            if dataset is not None:
                # Generate dataset
                hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
                mask_y = cv2.inRange(hsv, np.array([18, 50, 150]), np.array([40, 255, 255]))
                mask_w = cv2.inRange(hsv, np.array([0, 0, 200]),  np.array([180, 30, 255]))
                mask_c = np.zeros(mask_w.shape, np.uint8); mask_c[mask_w>0]=1; mask_c[mask_y>0]=2
                mask_rgb = np.zeros_like(rgb); mask_rgb[mask_c==1]=[255,255,255]; mask_rgb[mask_c==2]=[255,255,0]

                # You can get the controls of the vehicule at each snapshot
                ego_control = carla.VehicleControl()
                ego_loc = carla.Location(0,0,0)
                ego_yaw = 0.0
                
                if ego_actor is not None:
                     ego_control = ego_actor.get_control()
                     ego_loc = ego_actor.get_location()
                     ego_yaw = ego_actor.get_transform().rotation.yaw
                
                vru_loc = carla.Location(0,0,0)
                vru_yaw = 0.0
                if vru_actor is not None:
                     vru_loc = vru_actor.get_location()
                     vru_yaw = vru_actor.get_transform().rotation.yaw

                ego_throttle = float(ego_control.throttle)
                ego_steer    = max(-1.0, min(1.0, float(ego_control.steer)))
                ego_brake    = float(ego_control.brake)
                
                ego_x = ego_loc.x
                ego_y = ego_loc.y
                vru_x = vru_loc.x
                vru_y = vru_loc.y
                
                # placeholders for ego and vru speeds
                ego_speed = 0.0
                vru_speed = 0.0

                dataset.save_sample(rel_time, bgr, mask_rgb, 
                                    ego_throttle, ego_steer, ego_brake, 
                                    ego_speed, vru_speed,
                                    ego_x, ego_y, ego_yaw, vru_x, vru_y, vru_yaw)


    except KeyboardInterrupt:
        print("Exit...")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("Cleaning up...")
        if camera is not None:
            if camera.is_alive:
                camera.stop()
                camera.destroy()
            camera = None

        for agent in [vehicle, ego_actor, vru_actor]:    
            if agent is not None:
                try:
                    agent.destroy()
                except:
                    pass
                agent = None
        
        if dataset is not None:

            # Takes both dataset and speed CSV files and do the matching
            path = Path(args.log_path)
            logs = list(path.glob("*.csv"))    
            if (len(logs) == 0):
                print(f"Error, no data csv file found in {args.log_path}")
            else:
                csv_data_filename = str(logs[0])
                dataset.adjust_speed(csv_data_filename)


        pygame.quit()
        sys.exit()


if __name__ == "__main__":


    parser = argparse.ArgumentParser(description="recorder")

    parser.add_argument("--log_path", type=str, required=True,
                        help="Directory where log files will be loaded")
    
    parser.add_argument("--port", "--carla-port", type=int, default=3010,
                        help="Port used to connect to the CARLA simulator")
    
    parser.add_argument("--tport","--carla-traffic-port", type=int, default=3020,
                        help="Port used by the CARLA traffic manager")
    
    parser.add_argument(
                        "--dataset_types", "--carla-dataset-types",
                        nargs="+",
                        choices=["rgb", "mask", "segmented", "all"],
                        default=["all"],
                        metavar="TYPE",
                        help=(
                            "Types of frames to export. Options: rgb, mask, segmented, all. "
                            "Example: --dataset_types rgb mask"
                        )
                    )
    args = parser.parse_args()

    # Use "bike" or "car" to choose from where point of view you want to replay de simulation
    replay_loop(args, "car")

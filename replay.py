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
import os
import csv
import cv2
import pandas as pd
import queue
from queue import Queue

log_filename = "/tmp/town04_autopilot.log"
SPEED_CSV = "./speedTown4.csv"

# Dataset paths
currtime   = str(int(time.time() * 1000))
DATASET_ID = "Deepracer_BaseMap_" + currtime
SAVE_DIR   = DATASET_ID
RGB_DIR    = os.path.join(SAVE_DIR, "rgb")
MASK_DIR   = os.path.join(SAVE_DIR, "masks")
CSV_PATH   = os.path.join(SAVE_DIR, "dataset.csv")

os.makedirs(RGB_DIR, exist_ok=True)
os.makedirs(MASK_DIR, exist_ok=True)
if not os.path.exists(CSV_PATH):
    os.makedirs(os.path.dirname(CSV_PATH), exist_ok=True)
    with open(CSV_PATH, "w", newline="") as f:
        csv.writer(f).writerow(["rgb_path","mask_path","timestamp","throttle","steer","brake","speed"])


def load_speed_from_csv(
    dataset_csv: str,
    speed_csv: str,
    dst_speed_col: str = "speed",
    src_speed_col: str = "speed_m_s",
    src_time_col: str = "sim_time"
):

    if not os.path.isfile(dataset_csv):
        print(f" Unable to find dataset: {dataset_csv}")
        return
    if not os.path.isfile(speed_csv):
        print(f"Unable to find speed csv: {speed_csv}")
        return

    df_dst = pd.read_csv(dataset_csv)
    df_src = pd.read_csv(speed_csv)

    for col in ["timestamp", dst_speed_col]:
        if col not in df_dst.columns:
            print(f"Dataset does not have col: '{col}'.")
            return
    for col in [src_time_col, src_speed_col]:
        if col not in df_src.columns:
            print(f"CSV speed dataset does not have col: '{col}'.")
            return
    if df_dst.empty or df_src.empty:
        print("Warning: a file has empty data")
        return

    # Convert to num
    dst_ts = pd.to_numeric(df_dst["timestamp"], errors="coerce")
    src_ts = pd.to_numeric(df_src[src_time_col], errors="coerce")
    src_sp = pd.to_numeric(df_src[src_speed_col], errors="coerce")

    # Filter
    valid_src_mask = src_ts.notna() & src_sp.notna()
    if not valid_src_mask.any():
        print("Speed csv has incorrect data")
        return

    # 1) Keep first valid timestamp from speed csv
    first_src_idx = np.where(valid_src_mask.to_numpy())[0][0]
    first_src_time = float(src_ts.iloc[first_src_idx])

    # 2) Search same timestamp in the dataset to align both files 
    if dst_ts.notna().sum() == 0:
        print("Error finding timestamp")
        return

    # Fit index
    diffs = np.abs(dst_ts - first_src_time)
    anchor_dst_idx = int(diffs.idxmin())

    # 3) Prepare data to copy
    src_v = src_sp.iloc[first_src_idx:].to_numpy(dtype=float)

    # 4) Sequential copy
    n_dst = len(df_dst) - anchor_dst_idx
    n_src = len(src_v)
    n = min(n_dst, n_src)

    if n <= 0:
        print("Not enough space to copy speed")
        return

    df_dst.loc[anchor_dst_idx:anchor_dst_idx + n - 1, dst_speed_col] = src_v[:n]
    df_dst.to_csv(dataset_csv, index=False)

    # 5) Info
    print("[INFO] Fitting per timestamp:")
    print(f"  - first_src_time (vel CSV) = {first_src_time:.6f}")
    print(f"  - timestamp(dataset)[anchor] = {dst_ts.iloc[anchor_dst_idx]:.6f} (idx={anchor_dst_idx})")
    print(f"[OK] Cpied {n} speed data in '{dst_speed_col}' from {anchor_dst_idx} (sequential, ignored times from anchor.")


def safe_data(timestamp, bgr, mask_rgb, throttle, steer, brake, speed):
    rgb_name  = f"{timestamp}_rgb_{DATASET_ID}.png"
    mask_name = f"{timestamp}_mask_{DATASET_ID}.png"
    cv2.imwrite(os.path.join(RGB_DIR,  rgb_name),  bgr)
    cv2.imwrite(os.path.join(MASK_DIR, mask_name), cv2.cvtColor(mask_rgb, cv2.COLOR_RGB2BGR))
    with open(CSV_PATH, "a", newline="") as f:
        csv.writer(f).writerow([f"/rgb/{rgb_name}", f"/masks/{mask_name}", timestamp,
                                throttle, steer, brake, speed])


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

    # Synchronous mode disabled    
    # settings = world.get_settings()
    # settings.synchronous_mode = True
    # settings.fixed_delta_seconds = 0.05
    # world.apply_settings(settings)


    duration = get_log_duration(client, log_filename)
    duration = duration + world.get_snapshot().timestamp.elapsed_seconds
    print(f"Replaying: {log_filename}, duration: {duration:.2f} s")    
    

    client.replay_file(log_filename, 0, 0, 0)


    blueprint_library = world.get_blueprint_library()

    # Required to make a tick for actors to appear in the scene
    #world.tick()    
    
    actors = None

    if (view == "car"):
        actors = world.get_actors().filter("vehicle.tesla.model3")
    elif (view == "bike"):
        actors = world.get_actors().filter("vehicle.diamondback.century")

    if not actors:
        raise RuntimeError("No vehicles found in the replay")

    vehicle = actors[0]  # first vehicle is ego
    print(f"Using ego con id={vehicle.id}, type={vehicle.type_id}")
    
    
    
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

    # Start ata relative time 0.0 to syncronize with speed csv
    t0_sim = 0.0

    try:
        while True:            

            #world.tick() //synchronous_mode disabled
            clock.tick(30) # asynchronous mode enabled

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


            # Generate dataset
            hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
            mask_y = cv2.inRange(hsv, np.array([18, 50, 150]), np.array([40, 255, 255]))
            mask_w = cv2.inRange(hsv, np.array([0, 0, 200]),  np.array([180, 30, 255]))
            mask_c = np.zeros(mask_w.shape, np.uint8); mask_c[mask_w>0]=1; mask_c[mask_y>0]=2
            mask_rgb = np.zeros_like(rgb); mask_rgb[mask_c==1]=[255,255,255]; mask_rgb[mask_c==2]=[255,255,0]

            # You can get the controls of the vehicule at each snapshot
            # ctrl = vehicle.get_control()
            # print(ctrl.throttle, ctrl.steer, ctrl.brake)
            ctrl = vehicle.get_control()
            throttle = float(ctrl.throttle)
            steer    = max(-1.0, min(1.0, float(ctrl.steer)))
            brake    = float(ctrl.brake)
            speed = 0.0

            safe_data(rel_time, bgr, mask_rgb, throttle, steer, brake, speed)


    except KeyboardInterrupt:
        print("Exit...")

    finally:
        if camera is not None:
            camera.stop()
            camera.destroy()

        vehicle.destroy()

        # settings.synchronous_mode = False
        # world.apply_settings(settings)

        try:
            load_speed_from_csv(
                CSV_PATH,
                SPEED_CSV,
                dst_speed_col="speed",
                src_speed_col="speed_m_s"
            )
        except Exception as e:
            print(f"[ERROR] Acople secuencial de velocidades: {e}")

        pygame.quit()
        sys.exit()


if __name__ == "__main__":

    # Use "bike" or "car" to choose from where point of view you want to replay de simulation
    replay_loop("car")

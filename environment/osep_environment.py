#!/usr/bin/env python

import carb
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from pxr import Gf, UsdLux, Sdf

import os
import yaml
import omni
import numpy as np
import omni.isaac.core.utils.numpy.rotations as rot_utils
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.extensions import enable_extension
from scipy.spatial.transform import Rotation


# Import the Pegasus API for simulating drones
from pegasus.simulator.params import ROBOTS
from pegasus.simulator.logic.backends.px4_mavlink_backend import (
    PX4MavlinkBackend,
    PX4MavlinkBackendConfig,
)
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

from backend.sensor import StereoCamera, RTXLidar
from backend.ros_publishers import ClockPublisher, TfPublisher

enable_extension("omni.isaac.ros2_bridge")

simulation_app.update()


class PegasusApp:
    def __init__(self):
        self.working_dir = os.path.dirname(os.path.abspath(__file__))
        self.default_body_children = {"body", "base_link", "Looks"}

        self.topic_prefix = "/isaac"
        self.timeline = omni.timeline.get_timeline_interface()
        self.pg = PegasusInterface()

        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        self.setup_scene()
        self.world.reset()
        self.stop_sim = False

    def setup_scene(self):
        self.world.scene.add_default_ground_plane()
        ClockPublisher()
        self._spawn_ground_plane(scale=[500, 500, 500])
        self._spawn_light()
        self._spawn_windturbine(position=[200, 0, -0.75], filename="wind_turbine.usdc")
        self._spawn_quadrotor(position=[0, 0, 0], rotation=[0, 0, 0], vehicle_id=0)

    @staticmethod
    def _spawn_ground_plane(scale=[1000, 1000, 1000]):
        XFormPrim(prim_path="/World/defaultGroundPlane", scale=scale)
        return

    def _spawn_light(self):
        light = UsdLux.SphereLight.Define(self.world.stage, Sdf.Path("/World/Light"))
        light.CreateRadiusAttr(50.0)
        light.CreateIntensityAttr(1000.0)
        light.AddTranslateOp().Set(Gf.Vec3f(1000.0, 1000.0, 1000.0))
        return

    def _spawn_windturbine(self, position=[0.0, 0.0, -0.25], filename="windturbine.usdc"):
        data_dir = os.path.join(self.working_dir, "data/")
        windturbine_path = data_dir + filename
        add_reference_to_stage(
            usd_path=windturbine_path, prim_path="/World/Windturbine"
        )
        XFormPrim(
            prim_path="/World/Windturbine",
            position=position,
            scale=(0.3, 0.3, 0.3),
            orientation=rot_utils.euler_angles_to_quats(
                np.array([90.0, 0.0, 0.0]), degrees=True
            ),
        )
        return

    def _spawn_quadrotor(
        self,
        position=[0.0, 0.0, 0.07],
        rotation=[0.0, 0.0, 0.0],
        vehicle_id: int = 0,
        camera: bool = True,
        lidar: bool = True,
    ):
        if vehicle_id == 0:
            drone_prim_path="/World/quadrotor"
        else:
            drone_prim_path=f"/World/quadrotor_{vehicle_id}"
  
        config_multirotor = MultirotorConfig()
        # Create the multirotor configuration
        mavlink_config = PX4MavlinkBackendConfig(
            {
                "vehicle_id": vehicle_id,
                "px4_autolaunch": True,
                "px4_dir": self.pg.px4_path,
                "px4_vehicle_model": self.pg.px4_default_airframe,
            }
        )
        config_multirotor.backends = [PX4MavlinkBackend(mavlink_config)]

        Multirotor(
            drone_prim_path,
            ROBOTS["Iris"],
            vehicle_id,
            position,
            Rotation.from_euler("XYZ", rotation, degrees=True).as_quat(),
            config=config_multirotor,
        )

        config_file = self._load_config_file("sensor_config.yaml")
        
        if camera:
            StereoCamera(
                camera_config=config_file["stereo_camera"],
                topic_prefix=self.topic_prefix, 
                drone_prim_path=drone_prim_path, 
                vehicle_id=vehicle_id,
                translation=(0.1, 0.0, 0.2), 
            )

        if lidar:
            RTXLidar(
                lidar_config=config_file["rtx_lidar"],
                topic_prefix=self.topic_prefix,
                drone_prim_path=drone_prim_path,
                vehicle_id=vehicle_id,
                translation=(0.0, 0.0, 0.25),
            )
        TfPublisher(self.topic_prefix, drone_prim_path, self.default_body_children)
        return

    def _load_config_file(self, file_name):
        config_dir = os.path.join(self.working_dir, "config/")
        config_file_path = config_dir + file_name
        try:
            with open(config_file_path, "r") as file:
                config = yaml.safe_load(file)
        except FileNotFoundError:
            raise FileNotFoundError(f"File {config_file_path} not found")
        return config  

    def run(self):
        self.timeline.play()

        while simulation_app.is_running() and not self.stop_sim:
            self.world.step(render=True)

        carb.log_warn("PegasusApp Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()


def main():
    pg_app = PegasusApp()
    pg_app.run()


if __name__ == "__main__":
    main()

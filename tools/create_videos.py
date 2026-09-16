#!/usr/bin/env python3


import os
import math
import traceback

from pyrep import PyRep
from pyrep.const import RenderMode
from pyrep.robots.arms.dual_panda import PandaLeft, PandaRight
from pyrep.robots.end_effectors.dual_panda_gripper import PandaGripperLeft, PandaGripperRight
from pyrep.objects.shape import Shape

from yarr.utils.video_utils import CircleCameraMotion
from yarr.utils.video_utils import TaskRecorder

from pyrep.objects.dummy import Dummy
from pyrep.objects.vision_sensor import VisionSensor


from rlbench.backend.exceptions import *
from rlbench.observation_config import ObservationConfig
from rlbench.observation_config import CameraConfig

from rlbench.backend.robot import Robot
from rlbench.backend.robot import BimanualRobot
from rlbench.backend.scene import Scene

from rlbench.backend.task import BIMANUAL_TASKS_PATH
from rlbench.backend.const import BIMANUAL_TTT_FILE

import rich_click as click
from click_prompt import choice_option


from rlbench.utils import name_to_task_class


STATIC_CAMERA_VIEWS = [
    "front",
    "overhead",
    "over_shoulder_left",
    "over_shoulder_right",
    "wrist_left",
    "wrist_right",
]
CAMERA_VIEW_ALIASES = {
    "left_shoulder": "over_shoulder_left",
    "right_shoulder": "over_shoulder_right",
}
CAMERA_VIEW_CHOICES = ["cinematic", "all"] + STATIC_CAMERA_VIEWS + sorted(CAMERA_VIEW_ALIASES)


class StaticCameraMotion(object):
    def __init__(self, cam):
        self.cam = cam

    def step(self):
        self.render_current_frame()

    def render_current_frame(self):
        if self.cam.get_explicit_handling():
            self.cam.handle_explicitly()


class ExplicitCircleCameraMotion(CircleCameraMotion):
    def __init__(self, cam, origin, speed, init_rotation=math.radians(180)):
        self._origin_start_pose = origin.get_pose()
        super().__init__(cam, origin, speed, init_rotation)

    def step(self):
        super().step()
        self.render_current_frame()

    def render_current_frame(self):
        if self.cam.get_explicit_handling():
            self.cam.handle_explicitly()

    def restore_pose(self):
        self.origin.set_pose(self._origin_start_pose)


def parse_resolution(resolution):
    resolution = str(resolution).lower().replace(',', 'x')
    if 'x' in resolution:
        parts = resolution.split('x')
    else:
        parts = [resolution, resolution]

    if len(parts) != 2:
        raise click.BadParameter(
            'Resolution must be WIDTHxHEIGHT, WIDTH,HEIGHT, or a single square size.')

    try:
        width, height = [int(p) for p in parts]
    except ValueError as exc:
        raise click.BadParameter('Resolution values must be integers.') from exc

    if width <= 0 or height <= 0:
        raise click.BadParameter('Resolution values must be positive.')

    return [width, height]


def parse_xy(value, option_name):
    if value is None:
        return None

    parts = str(value).replace(',', ' ').split()
    if len(parts) != 2:
        raise click.BadParameter(f'{option_name} must be formatted as X,Y.')

    try:
        return [float(parts[0]), float(parts[1])]
    except ValueError as exc:
        raise click.BadParameter(f'{option_name} values must be numbers.') from exc


def render_current_frame(cam_motion):
    if hasattr(cam_motion, 'render_current_frame'):
        cam_motion.render_current_frame()
    else:
        cam_motion.step()


def save_recorded_frames(snaps, frames_output_dir):
    if not snaps:
        print(f"No frames to save to {frames_output_dir}")
        return

    os.makedirs(frames_output_dir, exist_ok=True)

    # OpenCV QT version can conflict with PyRep, so import here.
    import cv2

    for i, image in enumerate(snaps):
        frame = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        cv2.imwrite(os.path.join(frames_output_dir, f"frame_{i:04d}.png"), frame)

    print(f"Saving {len(snaps)} frames to {frames_output_dir}")


def set_scene_background_renderable(renderable):
    scene_background = [
        "Wall1", "Wall2", "Wall3", "Wall4", "Floor", "Roof",
        "ResizableFloor_5_25_visibleElement"]

    for object_name in scene_background:
        Shape(object_name).set_renderable(renderable)


def create_recording_camera(camera_view, camera_resolution, cinematic_options):
    if camera_view in STATIC_CAMERA_VIEWS:
        cam = VisionSensor(f"cam_{camera_view}")
        cam.set_explicit_handling(1)
        cam.set_resolution(camera_resolution)
        cam.set_render_mode(RenderMode.OPENGL3)
        return cam, StaticCameraMotion(cam)

    cam_placeholder = Dummy('cam_cinematic_placeholder')
    cam_base = Dummy('cam_cinematic_base')
    cam = VisionSensor.create(
        camera_resolution, explicit_handling=True,
        background_color=[1.0, 1.0, 1.0])
    cam.set_pose(cam_placeholder.get_pose())

    position = cam.get_position()
    if cinematic_options['camera_xy'] is None:
        position[0] += cinematic_options['xy_offset'][0]
        position[1] += cinematic_options['xy_offset'][1]
    else:
        position[0], position[1] = cinematic_options['camera_xy']
    if cinematic_options['camera_z'] is None:
        position[2] += cinematic_options['z_offset']
    else:
        position[2] = cinematic_options['camera_z']
    cam.set_position(position)

    orientation = cam.get_orientation()
    orientation += cinematic_options['euler_offset']
    cam.set_orientation(orientation)

    cam.set_parent(cam_placeholder)
    return cam, ExplicitCircleCameraMotion(
        cam, cam_base, 0.005,
        init_rotation=math.radians(cinematic_options['orbit_deg']))


def resolve_camera_views(camera_view):
    if camera_view == "all":
        return STATIC_CAMERA_VIEWS
    return [CAMERA_VIEW_ALIASES.get(camera_view, camera_view)]


def main():
    render_videos_for_task()


def get_bimanual_tasks():
    return [t.replace('.py', '') for t in
    os.listdir(BIMANUAL_TASKS_PATH) if t != '__init__.py' and t.endswith('.py')]


@click.command()
@choice_option('--bimanual-task-files', type=click.Choice(sorted(get_bimanual_tasks())), multiple=True)
@click.option('--add-task-description/--no-task-description', default=False)
@click.option('--add-gripper-color/--no-gripper-color', default=False)
@click.option('--resolution', default='1920x1200', show_default=True,
              help='Recording camera resolution: WIDTHxHEIGHT or a single square size.')
@click.option('--camera-view', default='cinematic', show_default=True,
              type=click.Choice(CAMERA_VIEW_CHOICES),
              help='Camera used for recording or first-frame capture.')
@click.option('--fps', default=30, show_default=True, type=int)
@click.option('--output-dir', default='/tmp', show_default=True,
              type=click.Path(file_okay=False, dir_okay=True),
              help='Root directory for success/failed video outputs.')
@click.option('--save-frames/--no-save-frames', default=False, show_default=True,
              help='Save every recorded RGB frame as a PNG sequence.')
@click.option('--first-frame-only/--full-demo', default=False, show_default=True,
              help='Capture the initialized scene frame and skip demo execution.')
@click.option('--frames-dir', default=None,
              type=click.Path(file_okay=False, dir_okay=True),
              help='Frame output root. Defaults depend on full-demo vs first-frame mode.')
@click.option('--cinematic-camera-xy', default=None,
              help='Absolute world X,Y for the cinematic camera. Keeps the original height.')
@click.option('--cinematic-xy-offset', default='0,0', show_default=True,
              help='World X,Y offset added to the default cinematic camera position.')
@click.option('--cinematic-camera-z', default=None, type=float,
              help='Absolute world Z for the cinematic camera.')
@click.option('--cinematic-z-offset', default=0.0, show_default=True, type=float,
              help='World Z offset added to the default cinematic camera height.')
@click.option('--cinematic-orbit-deg', default=180.0, show_default=True, type=float,
              help='Initial rotation around cam_cinematic_base.')
@click.option('--cinematic-roll-deg', default=0.0, show_default=True, type=float,
              help='Extra cinematic camera Euler-X angle.')
@click.option('--cinematic-pitch-deg', default=0.0, show_default=True, type=float,
              help='Extra cinematic camera Euler-Y angle.')
@click.option('--cinematic-yaw-deg', default=0.0, show_default=True, type=float,
              help='Extra cinematic camera Euler-Z angle.')
def render_videos_for_task(
        bimanual_task_files, add_task_description, add_gripper_color,
        resolution, camera_view, fps, output_dir, save_frames,
        first_frame_only, frames_dir, cinematic_camera_xy, cinematic_xy_offset,
        cinematic_camera_z, cinematic_z_offset, cinematic_orbit_deg,
        cinematic_roll_deg, cinematic_pitch_deg, cinematic_yaw_deg):

    CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
    camera_resolution = parse_resolution(resolution)
    camera_views = resolve_camera_views(camera_view)
    cinematic_options = {
        'camera_xy': parse_xy(cinematic_camera_xy, '--cinematic-camera-xy'),
        'xy_offset': parse_xy(cinematic_xy_offset, '--cinematic-xy-offset'),
        'camera_z': cinematic_camera_z,
        'z_offset': cinematic_z_offset,
        'orbit_deg': cinematic_orbit_deg,
        'euler_offset': [
            math.radians(cinematic_roll_deg),
            math.radians(cinematic_pitch_deg),
            math.radians(cinematic_yaw_deg),
        ],
    }

    if len(camera_views) > 1 and not first_frame_only:
        raise click.ClickException('--camera-view all is only supported with --first-frame-only.')

    pr = PyRep()
    ttt_file = os.path.join(CURRENT_DIR, '..', 'rlbench', BIMANUAL_TTT_FILE)
    pr.launch(ttt_file, responsive_ui=True)
    pr.step_ui()

    robot = BimanualRobot(PandaRight(), PandaGripperRight(), PandaLeft(), PandaGripperLeft())

    if add_gripper_color:
        robot.colorize_gripper()

    camera_configs = {
        camera_name: CameraConfig(
            rgb=True, depth=False, point_cloud=False, mask=False,
            image_size=camera_resolution, render_mode=RenderMode.OPENGL3)
        for camera_name in STATIC_CAMERA_VIEWS
    }
    obs_config = ObservationConfig()
    obs_config.set_all(False)
    obs_config.camera_configs = camera_configs

    scene = Scene(pr, robot, obs_config)

    for task_file in bimanual_task_files:


        if add_task_description:
            task_name_mapping = {}
            task_description = task_name_mapping.get(task_file, f"Demo for task {task_file}")
        else:
            task_description = ""

        task_class = name_to_task_class(task_file, True)
        task = task_class(pr, robot,  task_file.replace('.py', ''))
        try:
            print(f"Loading {task_file}")
            scene.load(task)
            pr.start()
            pr.step_ui()
            
            record_demo(
                scene, task, task_file, task_description, camera_resolution,
                camera_views, fps, output_dir, save_frames, first_frame_only,
                frames_dir, cinematic_options)
            scene.reset()
            scene.unload()

            pr.stop()
            pr.step_ui()


        except Exception as e:
            print("error while loading task")
            print(e)


    pr.step_ui()

    pr.stop()
    pr.shutdown()


def record_demo(
        scene, task, task_file, task_description, camera_resolution,
        camera_views, fps, output_dir, save_frames, first_frame_only,
        frames_dir, cinematic_options):

    if first_frame_only:
        hide_background = camera_views == ["cinematic"]
        if hide_background:
            set_scene_background_renderable(False)
        try:
            scene.init_task()
            scene.init_episode(scene._variation_index, randomly_place=False)
            frame_root = frames_dir or os.path.join(output_dir, "first_frames")

            for camera_view in camera_views:
                _, cam_motion = create_recording_camera(
                    camera_view, camera_resolution, cinematic_options)
                try:
                    render_current_frame(cam_motion)
                    snap = (cam_motion.cam.capture_rgb() * 255.).astype('uint8')
                    if len(camera_views) > 1:
                        camera_frame_root = os.path.join(frame_root, camera_view)
                    else:
                        camera_frame_root = frame_root
                    save_recorded_frames([snap], os.path.join(camera_frame_root, task_file))
                finally:
                    if hasattr(cam_motion, 'restore_pose'):
                        cam_motion.restore_pose()
            return
        finally:
            if hide_background:
                set_scene_background_renderable(True)

    camera_view = camera_views[0]
    _, cam_motion = create_recording_camera(
        camera_view, camera_resolution, cinematic_options)
    hide_background = camera_view == "cinematic"
    if hide_background:
        set_scene_background_renderable(False)

    try:
        tr = TaskRecorder(None, cam_motion, fps=fps)

        try:
            scene.get_demo(False, callable_each_step=tr.take_snap, randomly_place=False)
        except (WaypointError, NoWaypointsError, DemoError, Exception) as e:
            traceback.print_exc()
            return
        success, terminate = task.success()

        task_result = "success" if success else "failed"

        recording_output_path = os.path.join(
            output_dir, task_result, f"rlbench_video_{task_file}.mp4")

        os.makedirs(os.path.dirname(recording_output_path), exist_ok=True)

        if save_frames:
            task_frames_dir = frames_dir or os.path.join(
                output_dir, task_result, "frames")
            save_recorded_frames(tr._current_snaps, os.path.join(task_frames_dir, task_file))

        tr.save(recording_output_path, task_description, None)

        print(f"Saving video to {recording_output_path}")

        if success:
            print("Demo was a success!")
    finally:
        if hasattr(cam_motion, 'restore_pose'):
            cam_motion.restore_pose()
        if hide_background:
            set_scene_background_renderable(True)


if __name__ == '__main__':
    main()

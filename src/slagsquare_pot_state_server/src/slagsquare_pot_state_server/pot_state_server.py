""" pot_state_server.py - Contains the PotStateServer class that updates the
    pot states given detection pots in visual image, an ir image and
    homographies for transforming pixels between visual and ir image or between
    visual image and CAD
"""

import os
from threading import Lock
import numpy as np

import rospy
import rospkg
from actionlib import SimpleActionServer
from cv_bridge import CvBridge
import message_filters
from sensor_msgs.msg import Image, Temperature
from nav_msgs.msg import OccupancyGrid
from smelter_msgs.msg import DetectedObjects, PotState, PotStateArray, \
    PanTiltZoom
from smelter_actions.msg import UpdatePotStatesAction, UpdatePotStatesResult, \
    UpdatePotStatesGoal

from .grid_model import GridModel
from .pot_state_server_utils import contour_msg_to_array, \
    get_approximate_poses, is_close_enough, save_latest_grid_state, \
    load_latest_grid_state

from .crane_observer_wrapper import CraneObserverWrapper

class PotStateServer:
    """ Class that implements a server for pot states

    Processes pot detections and crane readings to update a grid model with
    occupancy, temperature, and weight info

    Attributes:
        grid (GridModel): The instance of a grid model
        grid_model_lock (Lock): Ensures mutual exclusion for grid model updates
        camera_poses (PanTiltZoom): The fixed camera poses of a patrol
        config (dict): Contains various configuration parameters
        latest_update_stamp (rospy.Time): The timestamp of the latest update
        update_sas (SimpleActionServer): The action server that receives update
        requests
        pubs (dict): Contains ROS publishers for various outputs
    """

    def __init__(self) -> None:
        """ Initializes the server by loading params and initializing the ROS
            interfaces and the GridModel
        """

        # this lock ensures that the grid model is not updated at the same time
        # by both the pot state server and the crane observer
        self.grid = None
        self.grid_model_lock = Lock()

        # load params
        self.camera_poses = rospy.get_param('~ptz_presets')
        grid_rows = rospy.get_param('~grid_rows')
        grid_cols = rospy.get_param('~grid_cols')
        self.config = {
            'auto_mode': rospy.get_param('~auto_mode', False),
            'mock': rospy.get_param('~mock', False),
            'crane_filepath': rospy.get_param('~crane_filepath', ''),
            'col_idx_offset': rospy.get_param('~column_indexing_offset'),
            'pickle_states': rospy.get_param('~pickle_states', False),
            'pickle_expiration_time': rospy.get_param(
                '~pickle_expiration_time', 30),
            'cranes': rospy.get_param('~cranes'),
            'use_proxy': rospy.get_param('~use_proxy'),
            'osi_pi': {
                'PIWEBAPI_URL': rospy.get_param('~piwebapi_url'),
                'AF_SERVER_NAME': rospy.get_param('~af_database_name'),
                'USER_NAME': rospy.get_param('~piwebapi_user'),
                'USER_PASSWORD': rospy.get_param('~piwebapi_pass'),
                'AUTH_TYPE': rospy.get_param('~piwebapi_security_method')
            }
        }

        # initialize vars
        self.initialize_grid(grid_rows=grid_rows,
                             grid_cols=grid_cols-self.config['col_idx_offset'])
        self.latest_update_stamp = rospy.Time.now()

        # initialize action server
        if self.config['auto_mode']:
            self.subs = {}
            self.subs['ir_img'] = message_filters.Subscriber(
                'ir/image_raw', Image, queue_size=1)
            self.subs['detected_pots'] = message_filters.Subscriber(
                'detected_pots', DetectedObjects, queue_size=1)
            self.subs['camera_pose'] = message_filters.Subscriber(
                'ptz/state', PanTiltZoom, queue_size=1)
            self.sync = message_filters.ApproximateTimeSynchronizer(
                list(self.subs.values()), 100, 30.0)
            self.sync.registerCallback(self.update_pot_states_callback)
        else:
            self.update_sas = SimpleActionServer(
                'update_pot_states', UpdatePotStatesAction,
                execute_cb=self.update_pot_states_action_callback,
                auto_start=False)
            self.update_sas.start()

        # pubs
        self.pubs = {
            'pot_states': rospy.Publisher(
                'pot_states', PotStateArray, queue_size=10),
            'ogm': rospy.Publisher(
                'occupancy_grid/map', OccupancyGrid, queue_size=10),
            'centers_img': rospy.Publisher(
                'centers_img', Image, queue_size=10),
            'ir_pots_img': rospy.Publisher(
                'ir/detected_pots/image_raw', Image, queue_size=10),
        }

        try:
            self.initialize_crane_observer()
        except Exception as exc:
            rospy.logerr(
                f'Failed to initialize crane observer with error: {exc}')


    def initialize_grid(self, grid_rows: int, grid_cols: int) -> None:
        """ Initializes the GridModel

            Args:
                grid_rows(int): The number of rows of the grid model
                grid_cols(int): The number of cols of the grid model
        """
        pkg_path = rospkg.RosPack().get_path('slagsquare_pot_state_server')
        config_path = os.path.join(pkg_path, 'config')
        media_path = os.path.join(pkg_path, 'media')
        h_ir2rgb_path = os.path.join(config_path, 'homographies_ir_rgb.yml')
        h_rgb2cad_path = os.path.join(config_path, 'homographies_rgb_cad.yml')

        loaded = False

        if self.config['pickle_states']:
            loaded, self.grid = load_latest_grid_state(
                self.config['pickle_expiration_time'])

        if not loaded:
            self.grid = GridModel(
                h_ir2rgb_path, h_rgb2cad_path, rows=grid_rows, cols=grid_cols)
            self.grid.setRgbParameters(480, 704)  # leave hardcoded for now
            cad_path = os.path.join(media_path, 'ground_plan.png')
            self.grid.setCadParameters(cad_path)
            self.grid.setOccupancyGrid()  # set rows and columns
            self.grid.setCraneGridMappingParameters(
                {
                    'x_start':  41.5,
                    'x_end':    83.5,
                    'y_start':  17.5,
                    'y_end':    1.3,
                }
            )


    def initialize_crane_observer(self) -> None:
        """ Initializes the crane observer and reader to fetch crane movements
        """

        # create an instance of the crane observer wrapper
        self.crane_observer = CraneObserverWrapper(
            name='h501',
            grid_models=[self.grid],
            config=self.config)

        # add crane reading in separate thread
        self.crane_timer = rospy.Timer(
            rospy.Duration(60), self.crane_observer_callback)

        # perform initial call
        self.crane_observer_callback(
            rospy.timer.TimerEvent(*[rospy.Time.now()]*5))


    def crane_observer_callback(self, event: rospy.timer.TimerEvent) -> None:
        """ Reads crane tags, processes them and extracts pot state changes

            Args:
                event (rospy.timer.TimerEvent): Timing info about callback
        """
        rospy.loginfo(f'Crane Observer timer called at {event.current_real}')

        # read new crane values for the past minute
        crane_df = self.crane_observer.read()

        if crane_df is None:
            rospy.logerr('Crane parameter reading failed')
            return

        # update the grid model based on the crane readings
        with self.grid_model_lock:
            self.crane_observer.update(crane_df)


    def is_first_pose(self, pose: tuple) -> bool:
        """ Returns true if the given pose is the first pose from the list of
            patrol poses

            Args:
                pose(tuple): Tuple containing Pan, Tilt, and Zoom values

            Returns:
                bool: True if pose is close enought to the first pose
        """
        first_pose = (self.camera_poses[0]['pan'],
                      self.camera_poses[0]['tilt'],
                      self.camera_poses[0]['zoom'])

        return is_close_enough(list(pose), list(first_pose), epsilon=1e-2)


    def update_pot_states_callback(
            self,
            ir_img: Image,
            rgb_pots: DetectedObjects,
            pose: PanTiltZoom) -> None:
        """ Message callback for updating the pot states

        Args:
            ir_img (Image): The latest ir shapshot from the camera
            rgb_pots (DetectedObjects): The detected pots in the rgb img
            pose (PanTiltZoom): The camera pose
        """
        rospy.logwarn('Updating pot states')
        try:
            with self.grid_model_lock:
                validation = self.update_pot_states(
                    ir_img, rgb_pots, pose)

            rospy.logwarn('Update of pot states succeeded with validation '
                          f'percentage={validation*100} %')
        except Exception as exc:
            rospy.logerr(f'Failed to update pot states with exception {exc}')


    def update_pot_states_action_callback(
            self,
            goal: UpdatePotStatesGoal) -> None:
        """ Action callback that executes a state update

        Args:
            goal (UpdataPotStatesGoal): The update goal from the action request
        """
        ir_img = goal.thermal_snapshot
        rgb_pots = goal.detected_pots
        pose = goal.camera_pose
        success = False
        validation = 0.0

        # update pot states
        try:
            with self.grid_model_lock:
                validation = self.update_pot_states(
                    ir_img, rgb_pots, pose)
            success = True
        except Exception as exc:
            rospy.logerr(f'Failed to update pot states with exception {exc}')
            success, validation = False, 0.0

        # prepare the action result and send it to the action client
        result = UpdatePotStatesResult()
        result.result = success
        result.pots_validation_percentage = validation
        if success:
            self.update_sas.set_succeeded(result)
        else:
            self.update_sas.set_aborted()


    def update_pot_states(self,
                          ir_img_msg: Image,
                          rgb_pots: PotStateArray,
                          cam_ptz: PanTiltZoom) -> int:
        """ Updates the pot states through the grid model and occupancy grid

            Args:
                ir_img_msg(Image) The IR image
                rgb_pots(PotStateArray) The pot contours from the rgb image
                cam_ptz(PanTiltZoom) -- The camera pan-tilt-zoom state

            Returns:
                int: The number of detected pots that were validated
        """
        rospy.logwarn('Updating pot states. Time elapsed since last update: '
                      '{}'.format((rospy.Time.now() -
                                   self.latest_update_stamp).to_sec()))

        # convert ir_img to cv format
        ir_img = CvBridge().imgmsg_to_cv2(ir_img_msg, "passthrough")

        # convert pots from msg to tuple
        rgb_contours = ()
        if len(rgb_pots.contours.contours) == 0:
            rospy.logwarn('No pot detections in latest frame')

        for contour in rgb_pots.contours.contours:
            rgb_contours += (contour_msg_to_array(contour),)

        # generated poses rounded to 2 decimal places upwards and downwards
        initial_pose = (cam_ptz.pan, cam_ptz.tilt, cam_ptz.ir_zoom)
        poses = get_approximate_poses(initial_pose)

        if self.is_first_pose(initial_pose):

            if self.grid.useColumnOffsets:
                rospy.logwarn('Updating column offsets')
                self.grid.columnOffsets = self.grid.calculateColumnOffsets()
                self.grid.validationMargin = 0.7
                self.grid.setPartialFunctions()
            else:
                self.grid.useColumnOffsets = True

        pose = None
        for pose in poses:
            if self.grid.checkPoseInHomographies(pose):
                break
        else:
            rospy.logwarn('Failed to find Homography for pose '
                          f'{initial_pose}. Tried poses {poses}')
            rospy.logwarn(
                'Available homographies: {}'.format(
                    self.grid.homographiesIrRgb.keys()))
            raise ValueError(f'Couldn\'t find homographies for pose {pose}')

        if len(ir_img.shape) == 3:
            ir_img = np.mean(ir_img, axis=2)

        # Update Grid States
        validation_count, ir_contours_img = \
            self.grid.updateState(pose, rgb_contours, ir_img)

        # publish centers image
        self.pubs['centers_img'].publish(
            CvBridge().cv2_to_imgmsg(self.grid.centersImg, encoding='rgb8'))

        # publish pot states
        self.pubs['pot_states'].publish(
            self.get_pot_states_msg(stamp=rgb_pots.header.stamp))

        # convert occupancy grid map to ROS message and publish
        self.pubs['ogm'].publish(self.get_ogm_msg(stamp=rgb_pots.header.stamp))

        # publish ir image annotated with contours
        self.pubs['ir_pots_img'].publish(
            CvBridge().cv2_to_imgmsg(ir_contours_img, encoding='bgr8'))

        # update timestamp
        self.latest_update_stamp = rospy.Time.now()
        if self.config['pickle_states']:
            save_latest_grid_state(self.grid)

        # calculate validation percentage of the pots
        validation = validation_count / len(rgb_contours)
        rospy.logwarn(f'cam_ptz: {pose}, detected_pots: {len(rgb_contours)},'
                      f'validation_count: {validation_count}')

        return validation


    def get_pot_states_msg(self, stamp: rospy.Time) -> PotStateArray:
        """ Creates a pot states msg based on the latest grid model

            Args:
                stamp(rospy.Time): The latest update timestamp

            Returns:
                PotStateArray: The new pot states in a ROS msg
        """
        pot_states = PotStateArray()
        pot_states.header.frame_id = 'map'
        pot_states.header.stamp = stamp
        pots_coords = np.array(np.nonzero(self.grid.potTemperatures))

        for y, x in pots_coords.T:
            pot_state = PotState()
            pot_state.header = pot_states.header
            pot_state.area = PotState.AREA_SLAGSQUARE
            pot_state.filling_level = PotState.FILLING_LEVEL_UNKNOWN
            pot_state.position_type = PotState.POSITION_TYPE_2D_GRID
            pot_state.composition = PotState.COMPOSITION_UNKNOWN
            pot_state.position.x = x + self.config['col_idx_offset']
            pot_state.position.y = y
            pot_state.temperature_history.append(
                Temperature(
                    header=pot_states.header,
                    temperature=self.grid.potTemperatures[y, x]))
            pot_states.pot_states.append(pot_state)

        return pot_states


    def get_ogm_msg(self, stamp: rospy.Time) -> OccupancyGrid:
        """ Initializes the occupancy grid map

            Args:
                stamp(rospy.Time): The timestamp of the latest occupancy grid

            Returns:
                OccupancyGrid: The occupancy grid in a ROS message
        """
        ogm_msg = OccupancyGrid()
        ogm_msg.header.stamp = stamp
        ogm_msg.header.frame_id = 'map'
        ogm_msg.info.map_load_time = ogm_msg.header.stamp
        ogm_msg.info.resolution = 1
        ''' TODO
            The +col_idx_offset is a temporary hack to allow the grid model to
            work with the 20 visible columns and the pot state server to
            publish 28 columns including the non-visible elevated cooling field
        '''
        ogm_msg.info.width = self.grid.cols + self.config['col_idx_offset']
        ogm_msg.info.height = self.grid.rows

        p_ogm = self.grid.occupancyMap.probabilityMap
        ogm_msg.data = \
            (100 * np.hstack(
                (np.zeros((ogm_msg.info.height,
                           self.config['col_idx_offset'])), p_ogm)
            ).ravel()).astype(np.int8)

        return ogm_msg

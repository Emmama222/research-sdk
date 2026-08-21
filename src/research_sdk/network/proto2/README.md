# Common : 
    Contains : Robot basic info : 
    1. Team color
    2. Division 
    3. id

# SSL - Vision 
    Contains : 
    1. Detection Data
        #### How it works :
        The ssl-vision can / most likely have more than one camera operating at the same time.
        There will be 4 different outputs from at most 4 different cameras, and they all have the same frame number, timecapture, timesent.

        Due to time and ability limitation, we should only consider where there's only one camera

        1. frame Data 
            - frame_number
            - t_capture 
            - t_sent
            - camera_id

        2. Detection of each ball (in balls): 
            - confidence
            - area
            - x
            - y
            - z
            - pixel_x
            - pixel_y
        3. Detection of each Robot (in each team) [robot_yellow,robot_blue]
            - confidence
            - robot_id
            - x 
            - y
            - orientation (o)
            - pixel_x
            - pixel_y
            - height
    
    2. Geometry Data
        1. field
            1. field Lines :1. p1, p2 : x, y, 2. Thickness
                - Top Touch Line
                - Bottom Touch Line
                - Left Goal Line
                - Right Goal Line
                - Half way Line
                - Centre Line
                - Left Penalty Stretch
                - Right Penalty Stretch 
                - Left Field Left Penalty Stretch 
                - Left Field Right Penalty Stretch 
                - Right Field Right Penalty Stretch
                - Right Field Left Penalty Stretch 
                - Centre Circle
        2. Calibration : Calib
            - camera_id
            - focal_length
            - principal_point_x
            - principal_point_y
            - distortion
            - q0
            - q1 
            - q2
            - q3
            - q4
            - tx
            - ty
            - tz
            - derived_camera_world_tx
            - derived_camera_world_ty
            - derived_camera_world_tz

# gr Sim 
    1. Recieving Robot Status from grSim
        1. robot_id
        2. infrared  (bool)
        3. flat_kick (bool)
        4. chip_kick (bool)

    2. Sending packet to gr Sim
        a. commands : modifying individual robot's operation
            1. timestamp
            2. is team yellow (bool)
            3. **Robot Commands**
                **REQUIRED**
                1. id : robotID 
                2. kick speed x 
                3. kick speed z
                4. vel tangnt
                5. vel normal
                6. vel angular 
                7. spinner
                8. wheels speed
                **OPTIONAL**
                9. wheel 1 speed
                10. wheel 2 speed
                11. wheel 3 speed
                12. wheel 4 speed

        b. replacements : change object position on field
            1. Ball Replacement : **OPTIONAL**
                1. x
                2. y
                3. vx 
                4. vy
            2. Robot Replacement : 
                **REQUIRED**
                1. x 
                2. y
                3. dir
                4. id
                5. is yellow team (bool)
                **OPTIONAL**
                6. turn on (bool) 

            

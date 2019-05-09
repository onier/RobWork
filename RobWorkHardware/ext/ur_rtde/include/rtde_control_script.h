const std::string NEW_LINE= "\n"; const std::string QUOTATION = "\""; std::string UR_SCRIPT = "def rtde_control():" + NEW_LINE  + 
"" + NEW_LINE  + 
"    global force_mode_type = 2" + NEW_LINE  + 
"    global selection_vector = [0, 0, 0, 0, 0, 0]" + NEW_LINE  + 
"    global task_frame = p[0, 0, 0, 0, 0, 0]" + NEW_LINE  + 
"    global wrench = [0, 0, 0, 0, 0, 0]" + NEW_LINE  + 
"    global limits = [0, 0, 0, 0, 0, 0]" + NEW_LINE  + 
"    global is_servoing = 0" + NEW_LINE  + 
"    global is_speeding = 0" + NEW_LINE  + 
"    global servo_target = [0, 0, 0, 0, 0, 0]" + NEW_LINE  + 
"    global servo_time = 0.2" + NEW_LINE  + 
"    global servo_lookahead_time = 0.1" + NEW_LINE  + 
"    global servo_gain = 300" + NEW_LINE  + 
"" + NEW_LINE  + 
"    global speed_type = 0" + NEW_LINE  + 
"    global speed_target = [0, 0, 0, 0, 0, 0]" + NEW_LINE  + 
"    global speed_acceleration = 0.5" + NEW_LINE  + 
"    global speed_time = 0.5" + NEW_LINE  + 
"" + NEW_LINE  + 
"    global servo_thrd = 0" + NEW_LINE  + 
"    global speed_thrd = 0" + NEW_LINE  + 
"" + NEW_LINE  + 
"    thread speed_thread():" + NEW_LINE  + 
"        while (True):" + NEW_LINE  + 
"            if speed_type == 0:" + NEW_LINE  + 
"                if speed_time > 0:" + NEW_LINE  + 
"                    speedl(speed_target, a=speed_acceleration, t=speed_time)" + NEW_LINE  + 
"                else:" + NEW_LINE  + 
"                    speedl(speed_target, a=speed_acceleration)" + NEW_LINE  + 
"                end" + NEW_LINE  + 
"            else:" + NEW_LINE  + 
"                if speed_time > 0:" + NEW_LINE  + 
"                    speedj(speed_target, a=speed_acceleration, t=speed_time)" + NEW_LINE  + 
"                else:" + NEW_LINE  + 
"                    speedj(speed_target, a=speed_acceleration)" + NEW_LINE  + 
"                end" + NEW_LINE  + 
"            end" + NEW_LINE  + 
"        end" + NEW_LINE  + 
"    end" + NEW_LINE  + 
"" + NEW_LINE  + 
"    thread servo_thread():" + NEW_LINE  + 
"        while (True):" + NEW_LINE  + 
"            servoj(servo_target, t=servo_time, lookahead_time=servo_lookahead_time, gain=servo_gain)" + NEW_LINE  + 
"        end" + NEW_LINE  + 
"    end" + NEW_LINE  + 
"" + NEW_LINE  + 
"    def signal_ready(val):" + NEW_LINE  + 
"        write_output_integer_register(0, val)" + NEW_LINE  + 
"    end" + NEW_LINE  + 
"" + NEW_LINE  + 
"    def rtde_cmd():" + NEW_LINE  + 
"        return read_input_integer_register(0)" + NEW_LINE  + 
"    end" + NEW_LINE  + 
"" + NEW_LINE  + 
"    def process_cmd():" + NEW_LINE  + 
"        cmd = read_input_integer_register(0)" + NEW_LINE  + 
"" + NEW_LINE  + 
"        if cmd == 1:" + NEW_LINE  + 
"            textmsg("+QUOTATION+"movej"+QUOTATION+")" + NEW_LINE  + 
"            q = [0, 0, 0, 0, 0, 0]" + NEW_LINE  + 
"            q[0] = read_input_float_register(0)" + NEW_LINE  + 
"            q[1] = read_input_float_register(1)" + NEW_LINE  + 
"            q[2] = read_input_float_register(2)" + NEW_LINE  + 
"            q[3] = read_input_float_register(3)" + NEW_LINE  + 
"            q[4] = read_input_float_register(4)" + NEW_LINE  + 
"            q[5] = read_input_float_register(5)" + NEW_LINE  + 
"            velocity = read_input_float_register(6)" + NEW_LINE  + 
"            acceleration = read_input_float_register(7)" + NEW_LINE  + 
"            textmsg("+QUOTATION+"Target q:"+QUOTATION+")" + NEW_LINE  + 
"            textmsg(q)" + NEW_LINE  + 
"            movej(q, a=acceleration, v=velocity)" + NEW_LINE  + 
"            textmsg("+QUOTATION+"movej done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 2:" + NEW_LINE  + 
"            textmsg("+QUOTATION+"movej_ik"+QUOTATION+")" + NEW_LINE  + 
"            pose = p[0, 0, 0, 0, 0, 0]" + NEW_LINE  + 
"            pose[0] = read_input_float_register(0)" + NEW_LINE  + 
"            pose[1] = read_input_float_register(1)" + NEW_LINE  + 
"            pose[2] = read_input_float_register(2)" + NEW_LINE  + 
"            pose[3] = read_input_float_register(3)" + NEW_LINE  + 
"            pose[4] = read_input_float_register(4)" + NEW_LINE  + 
"            pose[5] = read_input_float_register(5)" + NEW_LINE  + 
"            velocity = read_input_float_register(6)" + NEW_LINE  + 
"            acceleration = read_input_float_register(7)" + NEW_LINE  + 
"            textmsg("+QUOTATION+"Target pose:"+QUOTATION+")" + NEW_LINE  + 
"            textmsg(pose)" + NEW_LINE  + 
"            q = get_inverse_kin(pose)" + NEW_LINE  + 
"            textmsg("+QUOTATION+"Target q:"+QUOTATION+")" + NEW_LINE  + 
"            textmsg(q)" + NEW_LINE  + 
"            movej(q, a=acceleration, v=velocity)" + NEW_LINE  + 
"            textmsg("+QUOTATION+"movej_ik done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 3:" + NEW_LINE  + 
"            textmsg("+QUOTATION+"movel"+QUOTATION+")" + NEW_LINE  + 
"            pose = p[0, 0, 0, 0, 0, 0]" + NEW_LINE  + 
"            pose[0] = read_input_float_register(0)" + NEW_LINE  + 
"            pose[1] = read_input_float_register(1)" + NEW_LINE  + 
"            pose[2] = read_input_float_register(2)" + NEW_LINE  + 
"            pose[3] = read_input_float_register(3)" + NEW_LINE  + 
"            pose[4] = read_input_float_register(4)" + NEW_LINE  + 
"            pose[5] = read_input_float_register(5)" + NEW_LINE  + 
"            velocity = read_input_float_register(6)" + NEW_LINE  + 
"            acceleration = read_input_float_register(7)" + NEW_LINE  + 
"            textmsg("+QUOTATION+"Target pose:"+QUOTATION+")" + NEW_LINE  + 
"            textmsg(pose)" + NEW_LINE  + 
"            movel(pose, a=acceleration, v=velocity)" + NEW_LINE  + 
"            textmsg("+QUOTATION+"movel done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 4:" + NEW_LINE  + 
"            textmsg("+QUOTATION+"movel_fk"+QUOTATION+")" + NEW_LINE  + 
"            q = [0, 0, 0, 0, 0, 0]" + NEW_LINE  + 
"            q[0] = read_input_float_register(0)" + NEW_LINE  + 
"            q[1] = read_input_float_register(1)" + NEW_LINE  + 
"            q[2] = read_input_float_register(2)" + NEW_LINE  + 
"            q[3] = read_input_float_register(3)" + NEW_LINE  + 
"            q[4] = read_input_float_register(4)" + NEW_LINE  + 
"            q[5] = read_input_float_register(5)" + NEW_LINE  + 
"            velocity = read_input_float_register(6)" + NEW_LINE  + 
"            acceleration = read_input_float_register(7)" + NEW_LINE  + 
"            textmsg("+QUOTATION+"Target q:"+QUOTATION+")" + NEW_LINE  + 
"            textmsg(q)" + NEW_LINE  + 
"            movel(q, a=acceleration, v=velocity)" + NEW_LINE  + 
"            textmsg("+QUOTATION+"movel_fk done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 5:" + NEW_LINE  + 
"            textmsg("+QUOTATION+"movec"+QUOTATION+")" + NEW_LINE  + 
"            pose_via = p[0, 0, 0, 0, 0, 0]" + NEW_LINE  + 
"            pose_via[0] = read_input_float_register(0)" + NEW_LINE  + 
"            pose_via[1] = read_input_float_register(1)" + NEW_LINE  + 
"            pose_via[2] = read_input_float_register(2)" + NEW_LINE  + 
"            pose_via[3] = read_input_float_register(3)" + NEW_LINE  + 
"            pose_via[4] = read_input_float_register(4)" + NEW_LINE  + 
"            pose_via[5] = read_input_float_register(5)" + NEW_LINE  + 
"            pose_to = p[0, 0, 0, 0, 0, 0]" + NEW_LINE  + 
"            pose_to[0] = read_input_float_register(6)" + NEW_LINE  + 
"            pose_to[1] = read_input_float_register(7)" + NEW_LINE  + 
"            pose_to[2] = read_input_float_register(8)" + NEW_LINE  + 
"            pose_to[3] = read_input_float_register(9)" + NEW_LINE  + 
"            pose_to[4] = read_input_float_register(10)" + NEW_LINE  + 
"            pose_to[5] = read_input_float_register(11)" + NEW_LINE  + 
"            velocity = read_input_float_register(12)" + NEW_LINE  + 
"            acceleration = read_input_float_register(13)" + NEW_LINE  + 
"            movec_mode = read_input_integer_register(1)" + NEW_LINE  + 
"            textmsg("+QUOTATION+"pose_via:"+QUOTATION+")" + NEW_LINE  + 
"            textmsg(pose_via)" + NEW_LINE  + 
"            textmsg("+QUOTATION+"pose_to:"+QUOTATION+")" + NEW_LINE  + 
"            textmsg(pose_to)" + NEW_LINE  + 
"            movec(pose_to, pose_via, a=acceleration, v=velocity, mode=movec_mode)" + NEW_LINE  + 
"            stopl(10)" + NEW_LINE  + 
"            textmsg("+QUOTATION+"movec done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 6:" + NEW_LINE  + 
"            textmsg("+QUOTATION+"force_mode_start"+QUOTATION+")" + NEW_LINE  + 
"            force_mode_type = read_input_integer_register(1)" + NEW_LINE  + 
"            selection_vector[0] = read_input_integer_register(2)" + NEW_LINE  + 
"            selection_vector[1] = read_input_integer_register(3)" + NEW_LINE  + 
"            selection_vector[2] = read_input_integer_register(4)" + NEW_LINE  + 
"            selection_vector[3] = read_input_integer_register(5)" + NEW_LINE  + 
"            selection_vector[4] = read_input_integer_register(6)" + NEW_LINE  + 
"            selection_vector[5] = read_input_integer_register(7)" + NEW_LINE  + 
"" + NEW_LINE  + 
"            task_frame[0] = read_input_float_register(0)" + NEW_LINE  + 
"            task_frame[1] = read_input_float_register(1)" + NEW_LINE  + 
"            task_frame[2] = read_input_float_register(2)" + NEW_LINE  + 
"            task_frame[3] = read_input_float_register(3)" + NEW_LINE  + 
"            task_frame[4] = read_input_float_register(4)" + NEW_LINE  + 
"            task_frame[5] = read_input_float_register(5)" + NEW_LINE  + 
"" + NEW_LINE  + 
"            wrench[0] = read_input_float_register(6)" + NEW_LINE  + 
"            wrench[1] = read_input_float_register(7)" + NEW_LINE  + 
"            wrench[2] = read_input_float_register(8)" + NEW_LINE  + 
"            wrench[3] = read_input_float_register(9)" + NEW_LINE  + 
"            wrench[4] = read_input_float_register(10)" + NEW_LINE  + 
"            wrench[5] = read_input_float_register(11)" + NEW_LINE  + 
"" + NEW_LINE  + 
"            limits[0] = read_input_float_register(12)" + NEW_LINE  + 
"            limits[1] = read_input_float_register(13)" + NEW_LINE  + 
"            limits[2] = read_input_float_register(14)" + NEW_LINE  + 
"            limits[3] = read_input_float_register(15)" + NEW_LINE  + 
"            limits[4] = read_input_float_register(16)" + NEW_LINE  + 
"            limits[5] = read_input_float_register(17)" + NEW_LINE  + 
"" + NEW_LINE  + 
"            force_mode(task_frame, selection_vector, wrench, force_mode_type, limits)" + NEW_LINE  + 
"            textmsg("+QUOTATION+"force_mode started"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 7:" + NEW_LINE  + 
"            textmsg("+QUOTATION+"force_mode_update"+QUOTATION+")" + NEW_LINE  + 
"            wrench[0] = read_input_float_register(0)" + NEW_LINE  + 
"            wrench[1] = read_input_float_register(1)" + NEW_LINE  + 
"            wrench[2] = read_input_float_register(2)" + NEW_LINE  + 
"            wrench[3] = read_input_float_register(3)" + NEW_LINE  + 
"            wrench[4] = read_input_float_register(4)" + NEW_LINE  + 
"            wrench[5] = read_input_float_register(5)" + NEW_LINE  + 
"            force_mode(task_frame, selection_vector, wrench, force_mode_type, limits)" + NEW_LINE  + 
"            textmsg("+QUOTATION+"force_mode_update done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 8:" + NEW_LINE  + 
"            textmsg("+QUOTATION+"force_mode_stop"+QUOTATION+")" + NEW_LINE  + 
"            end_force_mode()" + NEW_LINE  + 
"            stopl(10)" + NEW_LINE  + 
"            textmsg("+QUOTATION+"force_mode stopped"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 9:" + NEW_LINE  + 
"            textmsg("+QUOTATION+"zero_ftsensor"+QUOTATION+")" + NEW_LINE  + 
"            zero_ftsensor()" + NEW_LINE  + 
"            textmsg("+QUOTATION+"ftsensor zeroed"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 10:" + NEW_LINE  + 
"            textmsg("+QUOTATION+"speedj"+QUOTATION+")" + NEW_LINE  + 
"            qd = [0, 0, 0, 0, 0, 0]" + NEW_LINE  + 
"            qd[0] = read_input_float_register(0)" + NEW_LINE  + 
"            qd[1] = read_input_float_register(1)" + NEW_LINE  + 
"            qd[2] = read_input_float_register(2)" + NEW_LINE  + 
"            qd[3] = read_input_float_register(3)" + NEW_LINE  + 
"            qd[4] = read_input_float_register(4)" + NEW_LINE  + 
"            qd[5] = read_input_float_register(5)" + NEW_LINE  + 
"" + NEW_LINE  + 
"            enter_critical" + NEW_LINE  + 
"            speed_type = 1" + NEW_LINE  + 
"            speed_acceleration = read_input_float_register(6)" + NEW_LINE  + 
"            speed_time = read_input_float_register(7)" + NEW_LINE  + 
"            speed_target = qd" + NEW_LINE  + 
"            exit_critical" + NEW_LINE  + 
"" + NEW_LINE  + 
"            if is_speeding == 0:" + NEW_LINE  + 
"                is_speeding = 1" + NEW_LINE  + 
"                if speed_thrd == 0:" + NEW_LINE  + 
"                    global speed_thrd = run speed_thread()" + NEW_LINE  + 
"                end" + NEW_LINE  + 
"            end" + NEW_LINE  + 
"            textmsg("+QUOTATION+"speedj done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 11:" + NEW_LINE  + 
"            textmsg("+QUOTATION+"speedl"+QUOTATION+")" + NEW_LINE  + 
"            xd = [0, 0, 0, 0, 0, 0]" + NEW_LINE  + 
"            xd[0] = read_input_float_register(0)" + NEW_LINE  + 
"            xd[1] = read_input_float_register(1)" + NEW_LINE  + 
"            xd[2] = read_input_float_register(2)" + NEW_LINE  + 
"            xd[3] = read_input_float_register(3)" + NEW_LINE  + 
"            xd[4] = read_input_float_register(4)" + NEW_LINE  + 
"            xd[5] = read_input_float_register(5)" + NEW_LINE  + 
"            enter_critical" + NEW_LINE  + 
"            speed_type = 0" + NEW_LINE  + 
"            speed_acceleration = read_input_float_register(6)" + NEW_LINE  + 
"            speed_time = read_input_float_register(7)" + NEW_LINE  + 
"            speed_target = xd" + NEW_LINE  + 
"            exit_critical" + NEW_LINE  + 
"" + NEW_LINE  + 
"            if is_speeding == 0:" + NEW_LINE  + 
"                is_speeding = 1" + NEW_LINE  + 
"                if speed_thrd == 0:" + NEW_LINE  + 
"                    global speed_thrd = run speed_thread()" + NEW_LINE  + 
"                end" + NEW_LINE  + 
"            end" + NEW_LINE  + 
"" + NEW_LINE  + 
"            textmsg("+QUOTATION+"speedl done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 12:" + NEW_LINE  + 
"            textmsg("+QUOTATION+"servoj"+QUOTATION+")" + NEW_LINE  + 
"            q = [0, 0, 0, 0, 0, 0]" + NEW_LINE  + 
"            q[0] = read_input_float_register(0)" + NEW_LINE  + 
"            q[1] = read_input_float_register(1)" + NEW_LINE  + 
"            q[2] = read_input_float_register(2)" + NEW_LINE  + 
"            q[3] = read_input_float_register(3)" + NEW_LINE  + 
"            q[4] = read_input_float_register(4)" + NEW_LINE  + 
"            q[5] = read_input_float_register(5)" + NEW_LINE  + 
"            velocity = read_input_float_register(6)" + NEW_LINE  + 
"            acceleration = read_input_float_register(7)" + NEW_LINE  + 
"" + NEW_LINE  + 
"            enter_critical" + NEW_LINE  + 
"            servo_target = q" + NEW_LINE  + 
"            servo_time = read_input_float_register(8)" + NEW_LINE  + 
"            servo_lookahead_time = read_input_float_register(9)" + NEW_LINE  + 
"            servo_gain = read_input_float_register(10)" + NEW_LINE  + 
"            exit_critical" + NEW_LINE  + 
"" + NEW_LINE  + 
"            if is_servoing == 0:" + NEW_LINE  + 
"                is_servoing = 1" + NEW_LINE  + 
"                if servo_thrd == 0:" + NEW_LINE  + 
"                    global servo_thrd = run servo_thread()" + NEW_LINE  + 
"                end" + NEW_LINE  + 
"            end" + NEW_LINE  + 
"            textmsg("+QUOTATION+"servoj done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 13:" + NEW_LINE  + 
"            textmsg("+QUOTATION+"servoc"+QUOTATION+")" + NEW_LINE  + 
"            pose = p[0, 0, 0, 0, 0, 0]" + NEW_LINE  + 
"            pose[0] = read_input_float_register(0)" + NEW_LINE  + 
"            pose[1] = read_input_float_register(1)" + NEW_LINE  + 
"            pose[2] = read_input_float_register(2)" + NEW_LINE  + 
"            pose[3] = read_input_float_register(3)" + NEW_LINE  + 
"            pose[4] = read_input_float_register(4)" + NEW_LINE  + 
"            pose[5] = read_input_float_register(5)" + NEW_LINE  + 
"            velocity = read_input_float_register(6)" + NEW_LINE  + 
"            acceleration = read_input_float_register(7)" + NEW_LINE  + 
"            blend = read_input_float_register(8)" + NEW_LINE  + 
"            textmsg("+QUOTATION+"Target pose:"+QUOTATION+")" + NEW_LINE  + 
"            textmsg(pose)" + NEW_LINE  + 
"            servoc(pose, a=acceleration, v=velocity, r=blend)" + NEW_LINE  + 
"            stopj(20)" + NEW_LINE  + 
"            textmsg("+QUOTATION+"servoc done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 16:" + NEW_LINE  + 
"            textmsg("+QUOTATION+"speed_stop"+QUOTATION+")" + NEW_LINE  + 
"            enter_critical" + NEW_LINE  + 
"            is_speeding = 0" + NEW_LINE  + 
"            kill speed_thrd" + NEW_LINE  + 
"            speed_thrd = 0" + NEW_LINE  + 
"            if speed_type == 0:" + NEW_LINE  + 
"                stopl(10)" + NEW_LINE  + 
"            else:" + NEW_LINE  + 
"                stopj(10)" + NEW_LINE  + 
"            end" + NEW_LINE  + 
"            exit_critical" + NEW_LINE  + 
"            textmsg("+QUOTATION+"speed_stop done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 17:" + NEW_LINE  + 
"            textmsg("+QUOTATION+"servo_stop"+QUOTATION+")" + NEW_LINE  + 
"            enter_critical" + NEW_LINE  + 
"            is_servoing = 0" + NEW_LINE  + 
"            kill servo_thrd" + NEW_LINE  + 
"            servo_thrd = 0" + NEW_LINE  + 
"            exit_critical" + NEW_LINE  + 
"            stopj(20)" + NEW_LINE  + 
"            textmsg("+QUOTATION+"servo_stop done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 18:" + NEW_LINE  + 
"            textmsg("+QUOTATION+"set_payload"+QUOTATION+")" + NEW_LINE  + 
"            mass = read_input_float_register(0)" + NEW_LINE  + 
"            cog_x = read_input_float_register(1)" + NEW_LINE  + 
"            cog_y = read_input_float_register(2)" + NEW_LINE  + 
"            cog_z = read_input_float_register(3)" + NEW_LINE  + 
"            cog = [cog_x, cog_y, cog_z]" + NEW_LINE  + 
"            if cog_x == 0 and cog_y == 0 and cog_z == 0:" + NEW_LINE  + 
"                set_payload(mass, get_target_payload_cog())" + NEW_LINE  + 
"            else:" + NEW_LINE  + 
"                set_payload(mass, cog)" + NEW_LINE  + 
"            end" + NEW_LINE  + 
"            textmsg("+QUOTATION+"active payload:"+QUOTATION+")" + NEW_LINE  + 
"            textmsg(get_target_payload())" + NEW_LINE  + 
"            textmsg("+QUOTATION+"set_payload done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 19:" + NEW_LINE  + 
"            textmsg("+QUOTATION+"teach_mode"+QUOTATION+")" + NEW_LINE  + 
"            teach_mode()" + NEW_LINE  + 
"            textmsg("+QUOTATION+"teach_mode done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 20:" + NEW_LINE  + 
"            textmsg("+QUOTATION+"end_teach_mode"+QUOTATION+")" + NEW_LINE  + 
"            end_teach_mode()" + NEW_LINE  + 
"            textmsg("+QUOTATION+"end_teach_mode done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 21:" + NEW_LINE  + 
"            textmsg("+QUOTATION+"force_mode_set_damping"+QUOTATION+")" + NEW_LINE  + 
"            damping = read_input_float_register(0)" + NEW_LINE  + 
"            force_mode_set_damping(damping)" + NEW_LINE  + 
"            textmsg("+QUOTATION+"force_mode_set_damping done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 22:" + NEW_LINE  + 
"            textmsg("+QUOTATION+"force_mode_set_gain_scaling"+QUOTATION+")" + NEW_LINE  + 
"            scaling = read_input_float_register(0)" + NEW_LINE  + 
"            force_mode_set_gain_scaling(scaling)" + NEW_LINE  + 
"            textmsg("+QUOTATION+"force_mode_set_gain_scaling done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 255:" + NEW_LINE  + 
"            textmsg("+QUOTATION+"Received stop!"+QUOTATION+")" + NEW_LINE  + 
"        end" + NEW_LINE  + 
"" + NEW_LINE  + 
"        return cmd != 255" + NEW_LINE  + 
"    end" + NEW_LINE  + 
"" + NEW_LINE  + 
"    #" + NEW_LINE  + 
"    # RTDE Control script - Main loop" + NEW_LINE  + 
"    #" + NEW_LINE  + 
"" + NEW_LINE  + 
"    textmsg("+QUOTATION+"RTDE Control Script Loaded"+QUOTATION+")" + NEW_LINE  + 
"" + NEW_LINE  + 
"    # Initialize force torque sensor" + NEW_LINE  + 
"    zero_ftsensor()" + NEW_LINE  + 
"" + NEW_LINE  + 
"    # Initialize gain and damping for force mode to a more stable default" + NEW_LINE  + 
"    force_mode_set_gain_scaling(0.5)" + NEW_LINE  + 
"    force_mode_set_damping(0.025)" + NEW_LINE  + 
"" + NEW_LINE  + 
"    keep_running = True" + NEW_LINE  + 
"    executing_cmd = False" + NEW_LINE  + 
"    signal_ready(1)" + NEW_LINE  + 
"" + NEW_LINE  + 
"    while keep_running:" + NEW_LINE  + 
"        if rtde_cmd() == 0:" + NEW_LINE  + 
"            signal_ready(1)" + NEW_LINE  + 
"            executing_cmd = False" + NEW_LINE  + 
"        else:" + NEW_LINE  + 
"            signal_ready(0)" + NEW_LINE  + 
"            if not executing_cmd:" + NEW_LINE  + 
"                keep_running = process_cmd()" + NEW_LINE  + 
"            end" + NEW_LINE  + 
"            executing_cmd = True" + NEW_LINE  + 
"        end" + NEW_LINE  + 
"        sync()" + NEW_LINE  + 
"    end" + NEW_LINE  + 
"    textmsg("+QUOTATION+"RTDE Control Script Terminated"+QUOTATION+")" + NEW_LINE  + 
"end" + NEW_LINE  + 
"run program" + NEW_LINE  + 
"";
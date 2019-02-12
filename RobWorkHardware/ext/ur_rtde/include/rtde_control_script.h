const std::string NEW_LINE= "\n"; const std::string QUOTATION = "\""; std::string UR_SCRIPT = "def rtde_control():" + NEW_LINE  + 
"" + NEW_LINE  + 
"    global force_mode_type = 2" + NEW_LINE  + 
"    global selection_vector = [0, 0, 0, 0, 0, 0]" + NEW_LINE  + 
"    global task_frame = p[0, 0, 0, 0, 0, 0]" + NEW_LINE  + 
"    global wrench = [0, 0, 0, 0, 0, 0]" + NEW_LINE  + 
"    global limits = [0, 0, 0, 0, 0, 0]" + NEW_LINE  + 
"" + NEW_LINE  + 
"    def signal_ready():" + NEW_LINE  + 
"        write_output_integer_register(0, 2)" + NEW_LINE  + 
"    end" + NEW_LINE  + 
"" + NEW_LINE  + 
"    def set_executing_cmd():" + NEW_LINE  + 
"        write_output_integer_register(0, 0)" + NEW_LINE  + 
"    end" + NEW_LINE  + 
"" + NEW_LINE  + 
"    def clear_executing_cmd():" + NEW_LINE  + 
"        write_output_integer_register(0, 1)" + NEW_LINE  + 
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
"            acceleration = read_input_float_register(6)" + NEW_LINE  + 
"            time = read_input_float_register(7)" + NEW_LINE  + 
"            if time > 0:" + NEW_LINE  + 
"                speedj(qd, acceleration, time)" + NEW_LINE  + 
"            else:" + NEW_LINE  + 
"                speedj(qd, acceleration)" + NEW_LINE  + 
"            end" + NEW_LINE  + 
"            stopj(10)" + NEW_LINE  + 
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
"            acceleration = read_input_float_register(6)" + NEW_LINE  + 
"            time = read_input_float_register(7)" + NEW_LINE  + 
"            if time > 0:" + NEW_LINE  + 
"                speedl(xd, acceleration, time)" + NEW_LINE  + 
"            else:" + NEW_LINE  + 
"                speedl(xd, acceleration)" + NEW_LINE  + 
"            end" + NEW_LINE  + 
"            stopl(10)" + NEW_LINE  + 
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
"            time = read_input_float_register(8)" + NEW_LINE  + 
"            lookahead_time = read_input_float_register(9)" + NEW_LINE  + 
"            gain = read_input_float_register(10)" + NEW_LINE  + 
"            textmsg("+QUOTATION+"Target q:"+QUOTATION+")" + NEW_LINE  + 
"            textmsg(q)" + NEW_LINE  + 
"            servoj(q, a=acceleration, v=velocity, t=time, lookahead_time=lookahead_time, gain=gain)" + NEW_LINE  + 
"            stopj(20)" + NEW_LINE  + 
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
"" + NEW_LINE  + 
"        elif cmd == 255:" + NEW_LINE  + 
"            stopj(20)" + NEW_LINE  + 
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
"    keep_running = True" + NEW_LINE  + 
"    executing_cmd = False" + NEW_LINE  + 
"    signal_ready()" + NEW_LINE  + 
"" + NEW_LINE  + 
"    while keep_running:" + NEW_LINE  + 
"        if rtde_cmd() == 0:" + NEW_LINE  + 
"            clear_executing_cmd()" + NEW_LINE  + 
"            executing_cmd = False" + NEW_LINE  + 
"        else:" + NEW_LINE  + 
"            set_executing_cmd()" + NEW_LINE  + 
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
with Ada.Text_IO;
with Ada.Exceptions;
with Motion_Planner.PH_Beziers; use Motion_Planner.PH_Beziers;
with Motion_Planner;            use Motion_Planner;

package body Stepgen.Stepgen is

   use type Planner.Corners_Index;

   PP_Execution_Block : Planner.Execution_Block;

   Writer_Index : Command_Queue_Index := 0 with
     Volatile, Atomic;
   Reader_Index : Command_Queue_Index := 0 with
     Volatile, Atomic;

   Runner_Is_Idle : Boolean := True with
     Volatile, Atomic;

   task body Preprocessor is
      Current_Time     : Time := 0.0 * s;
      Last_Stepper_Pos : Stepper_Position;
   begin
      loop
         Planner.Dequeue (PP_Execution_Block);

         if PP_Execution_Block.N_Corners /= 0 then
            Last_Stepper_Pos :=
              Position_To_Stepper_Position
                (Point_At_T (PP_Execution_Block.Beziers (PP_Execution_Block.Beziers'First), 0.5));
         end if;

         for I in PP_Execution_Block.Feedrate_Profiles'Range loop
            declare
               Start_Curve_Half_Distance : constant Length :=
                 Distance_At_T (PP_Execution_Block.Beziers (I - 1), 1.0) -
                 Distance_At_T (PP_Execution_Block.Beziers (I - 1), 0.5);
               End_Curve_Half_Distance   : constant Length := Distance_At_T (PP_Execution_Block.Beziers (I), 0.5);
               Mid_Distance              : constant Length :=
                 abs
                 (Point_At_T (PP_Execution_Block.Beziers (I), 0.0) -
                  Point_At_T (PP_Execution_Block.Beziers (I - 1), 1.0));
            begin
               while Current_Time < Total_Time (PP_Execution_Block.Feedrate_Profiles (I)) loop
                  declare
                     Distance : Length :=
                       Distance_At_Time
                         (PP_Execution_Block.Feedrate_Profiles (I),
                          Current_Time,
                          PP_Execution_Block.Segment_Limits (I).Crackle_Max,
                          PP_Execution_Block.Corner_Velocity_Limits (I - 1));
                     Pos      : Scaled_Position;
                  begin
                     if Distance < Start_Curve_Half_Distance then
                        Pos :=
                          Point_At_Distance (PP_Execution_Block.Beziers (I - 1), Distance + Start_Curve_Half_Distance);
                     elsif Distance < Start_Curve_Half_Distance + Mid_Distance then
                        Pos :=
                          Point_At_T (PP_Execution_Block.Beziers (I - 1), 1.0) +
                          (Point_At_T (PP_Execution_Block.Beziers (I), 0.0) -
                             Point_At_T (PP_Execution_Block.Beziers (I - 1), 1.0)) *
                            ((Distance - Start_Curve_Half_Distance) / Mid_Distance);
                     else
                        Pos :=
                          Point_At_Distance
                            (PP_Execution_Block.Beziers (I), Distance - Start_Curve_Half_Distance - Mid_Distance);
                     end if;

                     declare
                        Stepper_Pos    : constant Stepper_Position        := Position_To_Stepper_Position (Pos);
                        Stepper_Offset : constant Stepper_Position_Offset := Stepper_Pos - Last_Stepper_Pos;
                        Command        : Full_Command;
                     begin
                        Last_Stepper_Pos := Stepper_Pos;

                        Command.Safe_Stop_After := False;

                        for J in Stepper_Name loop
                           if Stepper_Offset (J).Offset = 0 then
                              Command.Steppers (J) := (Dir => Forward, Time_Between_Steps => Interpolation_Time * 2);
                           else
                              Command.Steppers (J) :=
                                (Dir                => Stepper_Offset (J).Dir,
                                 Time_Between_Steps =>
                                   Interpolation_Time / Low_Level_Time_Type (Stepper_Offset (J).Offset));
                           end if;
                        end loop;

                        loop
                           exit when Writer_Index + 1 /= Reader_Index;
                        end loop;

                        Command_Queue (Writer_Index) := Command;
                     end;

                     Writer_Index := @ + 1;
                  end;

                  Current_Time := Current_Time + Low_Level_To_Time (Interpolation_Time);
               end loop;

               Current_Time := Current_Time - Total_Time (PP_Execution_Block.Feedrate_Profiles (I));
            end;
         end loop;

         Command_Queue (Writer_Index) :=
           (Steppers        => [others => (Dir => Forward, Time_Between_Steps => Interpolation_Time * 2)],
            Safe_Stop_After => True);
      end loop;
   exception
      when E : others =>
         Ada.Text_IO.Put_Line ("Exception in Stepgen preprocessor:");
         Ada.Text_IO.Put_Line (Ada.Exceptions.Exception_Information (E));
   end Preprocessor;

   Empty_Queue : exception;

   task body Runner is
      Command             : Full_Command;
      Next_Step           : array (Stepper_Name) of Low_Level_Time_Type;
      Command_Start_Time  : Low_Level_Time_Type;
      Empty_Queue_Is_Safe : Boolean := True;
   begin
      --  Give queue time to fill and time for command to be prepared.
      Command_Start_Time := Get_Time + Interpolation_Time * Low_Level_Time_Type (Command_Queue_Index'Last);

      loop
         if Reader_Index = Writer_Index then
            Runner_Is_Idle := True;

            if not Empty_Queue_Is_Safe then
               raise Empty_Queue with "Stepgen queue is empty in middle of move.";
            end if;

            loop
               exit when Reader_Index /= Writer_Index;
            end loop;

            Runner_Is_Idle     := False;
            --  Give queue time to fill and time for command to be prepared.
            Command_Start_Time := Get_Time + Interpolation_Time * Low_Level_Time_Type (Command_Queue_Index'Last);
         end if;

         Command      := Command_Queue (Reader_Index);
         Reader_Index := @ + 1;

         Empty_Queue_Is_Safe := Command.Safe_Stop_After;

         for I in Stepper_Name loop
            Next_Step (I) := Command_Start_Time + Command.Steppers (I).Time_Between_Steps / 2;
         end loop;

         loop
            exit when Get_Time >= Command_Start_Time;
         end loop;

         for I in Stepper_Name loop
            Set_Direction (I, Command.Steppers (I).Dir);
         end loop;

         declare
            T              : Low_Level_Time_Type;
            All_Steps_Done : Boolean;
         begin
            loop
               T              := Get_Time;
               All_Steps_Done := True;

               for I in Stepper_Name loop
                  if Next_Step (I) <= T then
                     Do_Step (I);
                     Next_Step (I) := @ + Command.Steppers (I).Time_Between_Steps;
                  end if;

                  if Next_Step (I) < Command_Start_Time + Interpolation_Time then
                     All_Steps_Done := False;
                  end if;
               end loop;

               exit when All_Steps_Done;
            end loop;
         end;

         Command_Start_Time := @ + Interpolation_Time;
      end loop;
   exception
      when E : others =>
         Ada.Text_IO.Put_Line ("Exception in Stepgen runner:");
         Ada.Text_IO.Put_Line (Ada.Exceptions.Exception_Information (E));
   end Runner;

   procedure Wait_Until_Idle is
   begin
      loop
         exit when Runner_Is_Idle and Reader_Index = Writer_Index;
         delay Duration (Low_Level_To_Time (Interpolation_Time) / s);
      end loop;
   end Wait_Until_Idle;

   function "-" (Left, Right : Stepper_Position) return Stepper_Position_Offset is
   begin
      return Result : Stepper_Position_Offset do
         for I in Stepper_Name loop
            if Left (I) < Right (I) then
               Result (I).Offset := Right (I) - Left (I);
               Result (I).Dir    := Backward;
            else
               Result (I).Offset := Left (I) - Right (I);
               Result (I).Dir    := Forward;
            end if;
         end loop;
      end return;
   end "-";

end Stepgen.Stepgen;

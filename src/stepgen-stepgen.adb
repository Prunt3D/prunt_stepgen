-----------------------------------------------------------------------------
--                                                                         --
--                   Part of the Prunt Motion Controller                   --
--                                                                         --
--            Copyright (C) 2024 Liam Powell (liam@prunt3d.com)            --
--                                                                         --
--  This program is free software: you can redistribute it and/or modify   --
--  it under the terms of the GNU General Public License as published by   --
--  the Free Software Foundation, either version 3 of the License, or      --
--  (at your option) any later version.                                    --
--                                                                         --
--  This program is distributed in the hope that it will be useful,        --
--  but WITHOUT ANY WARRANTY; without even the implied warranty of         --
--  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          --
--  GNU General Public License for more details.                           --
--                                                                         --
--  You should have received a copy of the GNU General Public License      --
--  along with this program.  If not, see <http://www.gnu.org/licenses/>.  --
--                                                                         --
-----------------------------------------------------------------------------

with Ada.Text_IO;
with Ada.Exceptions;
with Motion_Planner; use Motion_Planner;

package body Stepgen.Stepgen is

   use type Planner.Corners_Index;

   PP_Execution_Block : Planner.Execution_Block;

   type Command_Queue_Index is mod 2**10;
   Command_Queue : array (Command_Queue_Index) of Full_Command with
     Volatile_Components;

   Writer_Index : Command_Queue_Index := 0 with
     Volatile, Atomic;
   Reader_Index : Command_Queue_Index := 0 with
     Volatile, Atomic;

   Runner_Is_Idle : Boolean := True with
     Volatile, Atomic;

   Hit_During_Accel : Boolean := False with
     Volatile, Atomic;

   type Atomic_Components_Position is new Position with
     Atomic_Components, Volatile_Components;
   PP_Last_Position : Atomic_Components_Position := Atomic_Components_Position (Initial_Position);

   Homing_Move_Data : Planner.Flush_Extra_Data_Type with
     Volatile, Atomic;

   function Apply_Step_Count_Delta (Pos : Stepper_Position) return Stepper_Position is
      Res : Stepper_Position;
   begin
      for I in Stepper_Name loop
         Res (I) := (Pos (I) / Step_Count_Delta) * Step_Count_Delta;
      end loop;
      return Res;
   end Apply_Step_Count_Delta;

   task body Preprocessor is
      Current_Time     : Time := 0.0 * s;
      Last_Stepper_Pos : Stepper_Position;
      Pos_Data         : Stepper_Pos_Data;

      type Homing_Move_When_Kind is (Not_Pending_Kind, This_Block_Kind, This_Move_Kind);
      Homing_Move_When : Homing_Move_When_Kind := Not_Pending_Kind;
   begin
      accept Setup (In_Pos_Data : Stepper_Pos_Data) do
         Pos_Data := In_Pos_Data;
      end Setup;

      Last_Stepper_Pos := Apply_Step_Count_Delta (Position_To_Stepper_Position (Initial_Position, Pos_Data));

      loop
         Planner.Dequeue (PP_Execution_Block);

         if Is_Homing_Move (Planner.Flush_Extra_Data (PP_Execution_Block)) then
            if PP_Execution_Block.N_Corners /= 2 then
               raise Constraint_Error with "Homing move must have exactly 2 corners.";
            end if;
            Wait_Until_Idle;
            Homing_Move_Data := Planner.Flush_Extra_Data (PP_Execution_Block);
            Homing_Move_When := This_Block_Kind;
         end if;

         for I in 2 .. PP_Execution_Block.N_Corners loop
            loop
               declare
                  Is_Past_Accel_Part : Boolean;
                  Pos                : constant Position                :=
                    Planner.Segment_Pos_At_Time (PP_Execution_Block, I, Current_Time, Is_Past_Accel_Part);
                  Stepper_Pos        : constant Stepper_Position        :=
                    Apply_Step_Count_Delta (Position_To_Stepper_Position (Pos, Pos_Data));
                  Stepper_Offset     : constant Stepper_Position_Offset := Stepper_Pos - Last_Stepper_Pos;
                  Command            : Full_Command;
               begin
                  case Homing_Move_When is
                     when This_Block_Kind =>
                        if Is_Past_Accel_Part then
                           Homing_Move_When := This_Move_Kind; --  Next loop iteration, not this one.
                        end if;
                        Command.Loop_Until_Hit := False;
                     when Not_Pending_Kind =>
                        Command.Loop_Until_Hit := False;
                     when This_Move_Kind =>
                        Command.Loop_Until_Hit := True;
                        Homing_Move_When       := Not_Pending_Kind;
                  end case;

                  PP_Last_Position := Atomic_Components_Position (Pos);

                  Last_Stepper_Pos := Stepper_Pos;

                  Command.Safe_Stop_After :=
                    I = PP_Execution_Block.N_Corners and Current_Time = Planner.Segment_Time (PP_Execution_Block, I);

                  for J in Stepper_Name loop
                     if Stepper_Offset (J).Offset = 0 then
                        Command.Steppers (J) := (Dir => Forward, N_Steps => 0);
                     else
                        Command.Steppers (J) := (Dir => Stepper_Offset (J).Dir, N_Steps => Stepper_Offset (J).Offset);
                     end if;
                  end loop;

                  loop
                     exit when Writer_Index + 1 /= Reader_Index;
                  end loop;

                  Command_Queue (Writer_Index) := Command;
               end;

               Writer_Index := @ + 1;

               if Homing_Move_When /= Not_Pending_Kind and Current_Time >= Planner.Segment_Time (PP_Execution_Block, I)
               then
                  raise Constraint_Error with "Homing move queued but end of block reached before execution.";
               end if;

               exit when Current_Time >= Planner.Segment_Time (PP_Execution_Block, I);

               if Homing_Move_When = This_Move_Kind then
                  Current_Time := Current_Time + Low_Level_To_Time (Loop_Interpolation_Time);
               else
                  Current_Time := Current_Time + Low_Level_To_Time (Interpolation_Time);
               end if;

               if I = PP_Execution_Block.N_Corners and Current_Time > Planner.Segment_Time (PP_Execution_Block, I) then
                  Current_Time := Planner.Segment_Time (PP_Execution_Block, I);
                  --  This is fine because the final bit of an execution block has very low velocity.
               end if;
            end loop;

            Current_Time := Current_Time - Planner.Segment_Time (PP_Execution_Block, I);
         end loop;

         Last_Stepper_Pos :=
           Apply_Step_Count_Delta
             (Position_To_Stepper_Position (Planner.Next_Block_Pos (PP_Execution_Block), Pos_Data));

         PP_Last_Position := Atomic_Components_Position (Planner.Next_Block_Pos (PP_Execution_Block));

         if Is_Homing_Move (Planner.Flush_Extra_Data (PP_Execution_Block)) then
            Wait_Until_Idle;
         end if;

         declare
            First_Accel_Distance : Length;
         begin
            if PP_Execution_Block.N_Corners < 2 then
               First_Accel_Distance := 0.0 * mm;
            else
               First_Accel_Distance := Planner.Segment_Accel_Distance (PP_Execution_Block, 2);
            end if;
            Finished_Block (Planner.Flush_Extra_Data (PP_Execution_Block), First_Accel_Distance, Hit_During_Accel);
         end;
      end loop;
   exception
      when E : others =>
         Ada.Text_IO.Put_Line ("Exception in Stepgen preprocessor:");
         Ada.Text_IO.Put_Line (Ada.Exceptions.Exception_Information (E));
   end Preprocessor;

   Empty_Queue : exception;

   task body Runner is
      Command                    : Full_Command;
      Next_Actual_Step           : array (Stepper_Name) of Low_Level_Time_Type;
      Next_Ideal_Step            : array (Stepper_Name) of Low_Level_Time_Type;
      Command_Start_Time         : Low_Level_Time_Type;
      Empty_Queue_Is_Safe        : Boolean                           := True;
      Last_Direction             : array (Stepper_Name) of Direction := [others => Forward];
      Last_Actual_Step           : array (Stepper_Name) of Low_Level_Time_Type;
      Params                     : Stepper_Parameters_Array;
      First_Command_Loop         : Boolean                           := True;
      Command_Interpolation_Time : Low_Level_Time_Type;
   begin
      accept Setup (In_Params : Stepper_Parameters_Array) do
         Params := In_Params;
      end Setup;

      for I in Stepper_Name loop
         Set_Direction (I, Last_Direction (I), Params (I).User_Data);
         Last_Actual_Step (I) := Get_Time;
      end loop;

      --  Give queue time to fill. This should also give plenty of time for direction setup unless someone runs this
      --  with silly parameters.
      Command_Start_Time := Get_Time + Interpolation_Time * Low_Level_Time_Type (Command_Queue_Index'Last);

      loop
         if Reader_Index = Writer_Index then
            Runner_Is_Idle := True;

            if (not Empty_Queue_Is_Safe) and (not Ignore_Empty_Queue) then
               raise Empty_Queue with "Stepgen queue is empty in middle of move.";
            end if;

            loop
               exit when Reader_Index /= Writer_Index;
            end loop;

            Runner_Is_Idle := False;

            if not Ignore_Empty_Queue then
               --  Give queue time to fill.
               Command_Start_Time := Get_Time + Interpolation_Time * Low_Level_Time_Type (Command_Queue_Index'Last);
            end if;
         end if;

         Command := Command_Queue (Reader_Index);

         if Command.Loop_Until_Hit then
            Command_Interpolation_Time := Loop_Interpolation_Time;
            if Is_Home_Switch_Hit (Homing_Move_Data) then
               Hit_During_Accel := First_Command_Loop;
               Reader_Index     := @ + 1;
            end if;
            First_Command_Loop := False;
         else
            Command_Interpolation_Time := Interpolation_Time;
            Reader_Index               := @ + 1;
            First_Command_Loop         := True;
         end if;

         Empty_Queue_Is_Safe := Command.Safe_Stop_After;

         for I in Stepper_Name loop
            Set_Direction (I, Command.Steppers (I).Dir, Params (I).User_Data);
         end loop;

         for I in Stepper_Name loop
            if Command.Steppers (I).N_Steps = 0 then
               Next_Ideal_Step (I) := Low_Level_Time_Type'Last;
            else
               Next_Ideal_Step (I) :=
                 Command_Start_Time +
                 Command_Interpolation_Time / Low_Level_Time_Type (2 * Command.Steppers (I).N_Steps);
            end if;
            Next_Actual_Step (I) := Next_Ideal_Step (I);
            if Last_Direction (I) /= Command.Steppers (I).Dir then
               Set_Direction (I, Command.Steppers (I).Dir, Params (I).User_Data);
               if Last_Actual_Step (I) + Params (I).Direction_Setup_Time > Next_Actual_Step (I) then
                  Next_Actual_Step (I) := Last_Actual_Step (I) + Params (I).Direction_Setup_Time;
               end if;
            end if;
            if Last_Actual_Step (I) + Params (I).Step_Time > Next_Actual_Step (I) then
               Next_Actual_Step (I) := Last_Actual_Step (I) + Params (I).Step_Time;
            end if;

            if Command.Steppers (I).N_Steps = 0 then
               Next_Actual_Step (I) := Low_Level_Time_Type'Last;
            end if;
         end loop;

         declare
            T              : Low_Level_Time_Type;
            All_Steps_Done : Boolean;
            N_Steps_Done   : array (Stepper_Name) of Natural_Step_Count := [others => 0];
         begin
            loop
               T              := Get_Time;
               All_Steps_Done := True;

               declare
                  Next_Time : Low_Level_Time_Type := Low_Level_Time_Type'Last;
               begin
                  for I in Stepper_Name loop
                     Next_Time := Low_Level_Time_Type'Min (Next_Time, Next_Actual_Step (I));
                  end loop;

                  if Next_Time /= Low_Level_Time_Type'Last then
                     Waiting_For_Time (Next_Time);
                  end if;
               end;

               for I in Stepper_Name loop
                  if Next_Actual_Step (I) <= T then
                     Do_Step (I, Params (I).User_Data);
                     T                    := Get_Time;
                     Last_Actual_Step (I) := T;
                     N_Steps_Done (I)     := @ + 1;
                     Next_Ideal_Step (I)  :=
                       Command_Start_Time +
                       Command_Interpolation_Time * Low_Level_Time_Type (2 * N_Steps_Done (I) + 1) /
                         Low_Level_Time_Type (2 * Command.Steppers (I).N_Steps);
                     --  Do not rearrange the above equation without carefully considering how it is computed with
                     --  integer arithmetic.
                     --
                     --  TODO: This assumes that Low_Level_Time_Type can be multipled by 2 * N_Steps without
                     --  overflowing. Document that Low_Level_Time_Type should be a fair bit larger than the
                     --  maximum possible value from Get_Time.
                     Next_Actual_Step (I) := Next_Ideal_Step (I);
                     if Last_Actual_Step (I) + Params (I).Step_Time > Next_Actual_Step (I) then
                        Next_Actual_Step (I) := Last_Actual_Step (I) + Params (I).Step_Time;
                     end if;
                  end if;

                  if N_Steps_Done (I) = Command.Steppers (I).N_Steps then
                     --  Ensure no extra steps are produced if some steppers are running behind.
                     Next_Actual_Step (I) := Low_Level_Time_Type'Last;
                  else
                     All_Steps_Done := False;
                  end if;
               end loop;

               exit when All_Steps_Done;
            end loop;
         end;

         Command_Start_Time := @ + Command_Interpolation_Time;
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

   function Last_Position return Position is
   begin
      return Position (PP_Last_Position);
   end Last_Position;

end Stepgen.Stepgen;

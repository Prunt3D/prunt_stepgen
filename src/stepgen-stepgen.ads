with Motion_Planner.Planner;
with Physical_Types; use Physical_Types;
with System.Multiprocessors;

generic
   type Low_Level_Time_Type is mod <>;
   --  Warning: The stepgen implementation currently assumes that this value will never overflow, which is fine for a
   --  MHz range 64-bit clock, but not fine for 32-bit or absurdly fast clocks.
   with function Time_To_Low_Level (T : Time) return Low_Level_Time_Type;
   with function Low_Level_To_Time (T : Low_Level_Time_Type) return Time;
   with function Get_Time return Low_Level_Time_Type;

   with package Planner is new Motion_Planner.Planner (<>);
   with function Is_Homing_Move (Data : Planner.Flush_Extra_Data_Type) return Boolean;
   with function Is_Home_Switch_Hit (Data : Planner.Flush_Extra_Data_Type) return Boolean;

   type Stepper_Name is (<>);
   type Stepper_Position is array (Stepper_Name) of Step_Count;

   type Stepper_Pos_Data is private;

   with function Position_To_Stepper_Position (Pos : Position; Data : Stepper_Pos_Data) return Stepper_Position;
   with function Stepper_Position_To_Position (Pos : Stepper_Position; Data : Stepper_Pos_Data) return Position;

   type Stepper_Output_Data is private;

   with procedure Do_Step (Stepper : Stepper_Name; Data : in out Stepper_Output_Data);
   with procedure Set_Direction (Stepper : Stepper_Name; Dir : Direction; Data : in out Stepper_Output_Data);

   with procedure Finished_Block
     (Data : Planner.Flush_Extra_Data_Type; First_Segment_Accel_Distance : Length; Hit_During_Accel : Boolean);

   Interpolation_Time : Low_Level_Time_Type;

   Initial_Position : Position;

   Preprocessor_CPU : System.Multiprocessors.CPU_Range;
   Runner_CPU : System.Multiprocessors.CPU_Range;

   Step_Count_Delta : Step_Count := 2;
   --  When keeping track of positions, the step generator will divide and then multiply positions by Step_Count_Delta.
   --  This is a simple way to support steppers that do not step on both edges as otherwise some sequences could cause
   --  unexpected results. An example is given below of a waveform that will generate back-and-forth movement when
   --  stepping on both edges but will only move in one direction when only stepping on one edge:
   --
   --  Step: ___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾
   --  Dir:  _____/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾
package Stepgen.Stepgen is

   --  Warning: The step generator is only guaranteed to stay idle inside of the Finished_Block callback. This
   --  procedure will work anywhere, but there is nothing keeping the step generator idle when outside of the callback.
   procedure Wait_Until_Idle;

   type Stepper_Parameters is record
      Direction_Setup_Time : Low_Level_Time_Type;
      Step_Time            : Low_Level_Time_Type;
      --  These limits will be enforced even if the commanded feedrate is faster, resulting in a discontinuity in
      --  acceleration if the limits are reached. The limits are also enforced if the step generator needs to catch up
      --  in case the step generation task is interrupted or is overloaded, so it is preferable to leave plenty of
      --  headroom between the maximum commanded feedrate and the maximum possible feedrate derived from these limits
      --  to avoid any issues in cases when catching up.
      --
      --  Step_Time is the minimum low or high time of each part of the step signal, not the time of the entire step.

      User_Data : Stepper_Output_Data;
   end record;

   type Stepper_Parameters_Array is array (Stepper_Name) of Stepper_Parameters;

   task Preprocessor with
     CPU => Preprocessor_CPU
   is
      entry Setup (In_Pos_Data : Stepper_Pos_Data);
      --  No commands will be generated until after Setup is called. All pending data will be processed after Setup is
      --  called;
   end Preprocessor;

   task Runner with
     CPU => Runner_CPU
   is
      entry Setup (In_Params : Stepper_Parameters_Array);
      --  No steps will be generated until after Setup is called. Commands can still be enqueued and will be executed
      --  after this entry is called.
   end Runner;

   function Last_Position return Position;

private

   subtype Natural_Step_Count is Step_Count range 0 .. Step_Count'Last;

   type Stepper_Position_Offset_Part is record
      Offset : Natural_Step_Count;
      Dir    : Direction;
   end record;

   type Stepper_Position_Offset is array (Stepper_Name) of Stepper_Position_Offset_Part;

   function "-" (Left, Right : Stepper_Position) return Stepper_Position_Offset;

   function Apply_Step_Count_Delta (Pos : Stepper_Position) return Stepper_Position;

   type Stepper_Command is record
      Dir                : Direction;
      N_Steps            : Natural_Step_Count;
      Time_Between_Steps : Low_Level_Time_Type;
   end record;

   type Stepper_Commands is array (Stepper_Name) of Stepper_Command;

   type Full_Command is record
      Steppers        : Stepper_Commands;
      Safe_Stop_After : Boolean;
      Loop_Until_Hit  : Boolean;
   end record;

end Stepgen.Stepgen;

with Motion_Planner.Planner;
with Physical_Types; use Physical_Types;

generic
   --  Warning: The stepgen implementation currently assumes that this value will never overflow, which is fine for a
   --  MHz range 64-bit clock, but not fine for 32-bit or absurdly fast clocks.
   type Low_Level_Time_Type is mod <>;
   with function Time_To_Low_Level (T : Time) return Low_Level_Time_Type;
   with function Low_Level_To_Time (T : Low_Level_Time_Type) return Time;
   with function Get_Time return Low_Level_Time_Type;

   with package Planner is new Motion_Planner.Planner (<>);
   with function Is_Homing_Move (Data : Planner.Flush_Extra_Data_Type) return Boolean;
   with function Is_Home_Switch_Hit (Data : Planner.Flush_Extra_Data_Type) return Boolean;

   type Step_Count is range <>;
   type Stepper_Name is (<>);
   type Stepper_Position is array (Stepper_Name) of Step_Count;

   with function Position_To_Stepper_Position (Pos : Scaled_Position) return Stepper_Position;

   --  Currently only suitable for double edge stepping.
   with procedure Do_Step (Stepper : Stepper_Name);
   with procedure Set_Direction (Stepper : Stepper_Name; Dir : Direction);

   with procedure Finished_Block (Data : Planner.Flush_Extra_Data_Type);

   Interpolation_Time : Low_Level_Time_Type;
package Stepgen.Stepgen is

   --  Warning: The step generator is only guaranteed to stay idle inside of the Finished_Block callback. This
   --  procedure will work anywhere, but there is nothing keeping the step generator idle when outside of the callback.
   procedure Wait_Until_Idle;

private

   task Preprocessor;
   task Runner;

   subtype Step_Count_Offset is
     Step_Count'Base range 0 .. Step_Count'Base'Max (abs Step_Count'Last, abs Step_Count'First);

   type Stepper_Position_Offset_Part is record
      Offset : Step_Count_Offset;
      Dir    : Direction;
   end record;

   type Stepper_Position_Offset is array (Stepper_Name) of Stepper_Position_Offset_Part;

   function "-" (Left, Right : Stepper_Position) return Stepper_Position_Offset;

   type Stepper_Command is record
      Dir                : Direction;
      Time_Between_Steps : Low_Level_Time_Type;
   end record;

   type Stepper_Commands is array (Stepper_Name) of Stepper_Command;

   type Full_Command is record
      Steppers        : Stepper_Commands;
      Safe_Stop_After : Boolean;
   end record;

   type Command_Queue_Index is mod 2**10;
   Command_Queue : array (Command_Queue_Index) of Full_Command with
     Volatile_Components;

end Stepgen.Stepgen;

package Stepgen is

   type Step_Count is range -(2**63 - 1) .. 2**63 - 1;
   type Direction is (Forward, Backward);

end Stepgen;

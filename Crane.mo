model Crane
   
  import PlanarMechanics.PlanarWorld;
  import PlanarMechanics.Parts.*;
  import PlanarMechanics.Joints.*;
  import Modelica.Units.SI.*;
  
  parameter Inertia inertia = 0.1; // Inertia of mass objects [kg*m^2]
  parameter Mass m_crane = 5;  // Crane arm weight [kg]
  parameter Length l_crane_arm_part = 1; // Crane arm length [m]. This is the lenght of each part so the total length is 3x this
  parameter Mass m_load = 10;  // Load weight [kg]
  parameter Length l_load_wire = 3;  // Lenth of wire holding the load [m]
  
  
  inner PlanarWorld planarWorld(defaultWidthFraction=10);  // World inside which we're executing the simulation

  // Create crane arm (we model it as 3 parts, so that we have 2 intermediate points along the arm where we can connect the springs and control wires)
  Fixed crane_arm_FP(phi=0);  // Fixed point for crane arm
  Revolute crane_arm_join; // Object to enable rotation of crane arm around the fixed point
  FixedTranslation crane_arm1(r={l_crane_arm_part*1, 0}, width=0.1); // Object to represent the first part of the arm
  FixedTranslation crane_arm2(r={l_crane_arm_part*2, 0}, width=0.1); // Object to represent the second part of the arm
  FixedTranslation crane_arm3(r={l_crane_arm_part*3, 0}, width=0.1); // Object to represent the third part of the arm
  Body crane_arm_weight(m=m_crane, I=inertia);  // Object to represent crane arm weight

  // Create a wire to hold the load (will be modeled as a rigid bar), has no weight
  FixedTranslation load_wire(r={l_load_wire, 0}, width=0.02); // Very small width to simulate thin wire
  Revolute crane_wire_join; // Object to enable rotation of wire attached to the end of the crane arm
  
  // Create load on the crane
  Body load(m=m_load, I=inertia, sphereDiameter=0.3);   // Crane load (mass point)

equation
   
  // Create crane arm with the wire
  connect(crane_arm_FP.frame, crane_arm_join.frame_a);  // Attach join to fixed point
  connect(crane_arm_join.frame_b, crane_arm1.frame_a);  // Attach first part of the arm to join
  connect(crane_arm1.frame_b, crane_arm2.frame_a);  // Attach second part of the arm to the first
  connect(crane_arm2.frame_b, crane_arm3.frame_a);  // Attach third part the arm to the second
  connect(crane_arm3.frame_b, crane_arm_weight.frame_a);  // Attach weight to the end of the arm (end of 3rd part)
  connect(crane_arm3.frame_b, crane_wire_join.frame_a);   // Attach join for the wire to the end of the arm (end of 3rd part)
  connect(crane_wire_join.frame_b, load_wire.frame_a); // Attach wire to the join at the end of the arm
  
  // Attach load to the wire
  connect(load.frame_a, load_wire.frame_b);


  annotation(experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-06, Interval = 0.01));

end Crane;




model Crane
   
  import PlanarMechanics.PlanarWorld;
  import PlanarMechanics.Parts.*;
  import PlanarMechanics.Joints.*;
  import PlanarMechanics.Sensors.*;
  import PlanarMechanics.Sources.*;
  import Modelica.Units.SI.*;
  import Modelica.Math.*;
  import Modelica.Constants.*;
  
  parameter Inertia inertia = 0.1; // Inertia of mass objects [kg*m^2]
  parameter Mass m_crane = 5;  // Crane arm weight [kg]
  parameter Length l_crane_arm = 3; // Crane arm length [m]. This is the lenght of each part so the total length is 3x this
  parameter Mass m_load = 0.01;  // Load weight [kg]
  parameter Length l_load_wire = 3;  // Lenth of wire holding the load [m]
  parameter TranslationalSpringConstant k_spring_top = 78; // Spring constant of top spring [N/m]
  parameter TranslationalSpringConstant k_spring_bottom = 5; // Spring constant of bottom spring [N/m]
  parameter Length fixed_point_offset = 2; // How far appart (in m) are the fixed points
  parameter Angle initial_desired_angle = 0; // The desired angle of the crane, 0 = horizontal [rad]
  parameter Real Kp = 20; // Proportional gain
  parameter Real Ki = 2;  // Integral gain
  parameter Real Kd = 5;   // Derivative gain


  inner PlanarWorld planarWorld(defaultWidthFraction=10);  // World inside which we're executing the simulation
  
  Fixed top_FP(r={0, fixed_point_offset});  // Top fixed point (top spring and control wire)
  Fixed middle_FP;  // Middle fixed point (crane arm)
  Fixed bottom_FP(r={0, -fixed_point_offset}); // Bottom fixed point (bottom spring and control wire)

  // Create crane arm (we model it as 3 parts, so that we have 2 intermediate points along the arm where we can connect the springs and control wires)
  Revolute crane_arm_join; // Object to enable rotation of crane arm around the fixed point
  FixedTranslation crane_arm1(r={l_crane_arm/3, 0}, width=0.1); // Object to represent the first part of the arm
  FixedTranslation crane_arm2(r={l_crane_arm/3, 0}, width=0.1); // Object to represent the second part of the arm
  FixedTranslation crane_arm3(r={l_crane_arm/3, 0}, width=0.1); // Object to represent the third part of the arm
  Body crane_arm_weight(m=m_crane, I=inertia);  // Object to represent crane arm weight

  // Create a wire to hold the load (will be modeled as a rigid bar), has no weight
  FixedTranslation load_wire(r={0, -l_load_wire}, width=0.02); // Very small width to simulate thin wire
  Revolute crane_wire_join; // Object to enable rotation of wire attached to the end of the crane arm
  
  // Create load on the crane
  Body load(m=m_load, I=inertia, sphereDiameter=0.3);   // Crane load (mass point)

  // Create spring that holds the crane arm from the top
  Spring spring_top(c_x=k_spring_top, c_y=k_spring_top, c_phi=k_spring_top);
  
  // Create spring that holds the crane arm from the bottom
  Spring spring_bottom(c_x=k_spring_bottom, c_y=k_spring_bottom, c_phi=k_spring_top);
  
  // Create force from top control wire
  Angle top_force_angle; // Angle at which the top force is acting on the crane arm [rad]
  WorldForce wire_top(force={-sin(top_force_angle)*top_wire_force, cos(top_force_angle)*top_wire_force, 0});
  
  // Create force from bottom control wire
  Angle bottom_force_angle; // Angle at which the bottom force is acting on the crane arm [rad]
  WorldForce wire_bottom(force={-sin(bottom_force_angle)*bottom_wire_force, -cos(bottom_force_angle)*bottom_wire_force, 0});

  Real crane_x;  // X coordinate of the end of the crane arm
  Real crane_y;  // Y coordinate of the end of the crane arm
  Angle crane_angle; // Angle of the crane (positive = above level, negative = below level) [rad]
  Force top_wire_force; // Force applied to the top wire
  Force bottom_wire_force; // Force applied to the bottom wire
  
  // Variables for PID controller
  Angle angle_error;
  Real integral_error(start=0);
  Real derivative_error;
  Force control_force;
  Real desired_angle(start=initial_desired_angle);
equation
   
  // Create crane arm with the wire
  connect(middle_FP.frame, crane_arm_join.frame_a);  // Attach join to fixed point
  connect(crane_arm_join.frame_b, crane_arm1.frame_a);  // Attach first part of the arm to join
  connect(crane_arm1.frame_b, crane_arm2.frame_a);  // Attach second part of the arm to the first
  connect(crane_arm2.frame_b, crane_arm3.frame_a);  // Attach third part the arm to the second
  connect(crane_arm3.frame_b, crane_arm_weight.frame_a);  // Attach weight to the end of the arm (end of 3rd part)
  connect(crane_arm3.frame_b, crane_wire_join.frame_a);   // Attach join for the wire to the end of the arm (end of 3rd part)
  connect(crane_wire_join.frame_b, load_wire.frame_a); // Attach wire to the join at the end of the arm
  
  // Attach load to the wire
  connect(load.frame_a, load_wire.frame_b);

  // Create top spring
  connect(spring_top.frame_a, top_FP.frame); // Attach spring to fixed point
  connect(crane_arm1.frame_b, spring_top.frame_b);  // Attach spring to crane arm
  
  // Create bottom spring
  connect(spring_bottom.frame_a, bottom_FP.frame);   // Attach spring to fixed point
  connect(crane_arm1.frame_b, spring_bottom.frame_b);  // Attach spring to crane arm
  
  // Determine crane location
  crane_x = crane_arm3.frame_b.x;
  crane_y = crane_arm3.frame_b.y;
  crane_angle = asin(crane_y/l_crane_arm);
  
  // Determine the angle at which the force from the top wire is acting on the arm and then create it
  top_force_angle = asin(fixed_point_offset * sin(pi/2+crane_angle) / sqrt(fixed_point_offset*fixed_point_offset + (l_crane_arm*2/3)*(l_crane_arm*2/3) - 2*(l_crane_arm*2/3)*fixed_point_offset*cos(pi/2+crane_angle)));
  connect(crane_arm2.frame_b, wire_top.frame_b);

  // Determine the angle at which the force from the bottom wire is acting on the arm and then create it
  bottom_force_angle = asin(fixed_point_offset * sin(pi/2-crane_angle) / sqrt(fixed_point_offset*fixed_point_offset + (l_crane_arm*2/3)*(l_crane_arm*2/3) - 2*(l_crane_arm*2/3)*fixed_point_offset*cos(pi/2-crane_angle)));
  connect(crane_arm2.frame_b, wire_bottom.frame_b);
  
  // Apply forces so that the crane angle is as close as possible to the desired angle using a PID control
  angle_error = desired_angle - crane_angle;
  der(integral_error) = angle_error;
  derivative_error = der(angle_error);
  control_force = Kp * angle_error + Ki * integral_error + Kd * derivative_error;
  top_wire_force = control_force / 2;   // Distribute force to the top wire
  bottom_wire_force = -control_force / 2; // Distribute force to the bottom wire
  
  // Control desired angle over time
  if time < 20 then
    desired_angle = 0; 
  elseif time < 40 then
    desired_angle = 10 * pi / 180;
  elseif time < 60 then
    desired_angle = 0;
  else
    desired_angle = -10 * pi / 180; 
  end if;

  annotation(experiment(StartTime = 0, StopTime = 100, Tolerance = 1e-06, Interval = 0.01));

end Crane;


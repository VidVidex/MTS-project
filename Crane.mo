package CraneFaultModel
  type FaultType = enumeration(ok, broken, overbent);
  
  model Joint
    extends PlanarMechanics.Joints.Revolute;
    FaultType state(start = FaultType.ok);
    Modelica.Units.SI.Angle measured_angle;
  equation
    if state == FaultType.ok then
      measured_angle = phi;
    elseif state == FaultType.broken then
      measured_angle = 0;//pre(measured_angle);
    else
      measured_angle = 0;
    end if;
  end Joint;
  
  model LoadAttachment
    extends PlanarMechanics.Joints.Revolute;
    FaultType state(start = FaultType.ok);
    Modelica.Units.SI.Force measured_weight;
  equation
    if state == FaultType.ok then
      measured_weight = sqrt(frame_b.fy^2 + frame_b.fx^2);
    elseif state == FaultType.overbent then
      measured_weight = 1e9;
    elseif state == FaultType.broken then
      measured_weight = 0;
    else
      measured_weight = 0;
    end if;
  end LoadAttachment;
  
  model PullWire
    extends PlanarMechanics.Sources.RelativeForce;
    FaultType state(start = FaultType.ok);
    Modelica.Blocks.Interfaces.RealInput wanted_force[3];
  equation
    if state == FaultType.ok then
      force = wanted_force;
    elseif state == FaultType.broken then
      force = {0, 0, 0};
    else
      force = {0, 0, 0};
    end if;
  end PullWire;
  
  model CraneSpring
    extends PlanarMechanics.Parts.Spring(c_y = k_spring, c_phi = 1e5, c_x = k_spring);
    FaultType state(start = FaultType.ok);
    parameter Modelica.Units.SI.TranslationalSpringConstant k_spring;
  equation
    //if state == FaultType.ok then
    //  c_x = k_spring;
    //elseif state == FaultType.broken then
    //  c_x = 0;
    //else
    //  c_x = 0;
    //end if;
  end CraneSpring;

  model Crane
    import PlanarMechanics.PlanarWorld;
    import PlanarMechanics.Parts.*;
    import PlanarMechanics.Joints.*;
    import PlanarMechanics.Sensors.*;
    import PlanarMechanics.Sources.*;
    import Modelica.Units.SI.*;
    import Modelica.Math.*;
    import Modelica.Constants.*;
    
    parameter Inertia inertia = 0.1;
    // Inertia of mass objects [kg*m^2]
    parameter Mass m_crane = 5;
    // Crane arm weight [kg]
    parameter Length l_crane_arm = 3;
    // Crane arm length [m]. This is the lenght of each part so the total length is 3x this
    parameter Mass m_load = 0.01;
    // Load weight [kg]
    parameter Length l_load_wire = 3;
    // Lenth of wire holding the load [m]
    parameter TranslationalSpringConstant k_spring_top = 78;
    // Spring constant of top spring [N/m]
    parameter TranslationalSpringConstant k_spring_bottom = 5;
    // Spring constant of bottom spring [N/m]
    parameter Length fixed_point_offset = 2;
    // How far appart (in m) are the fixed points
    parameter Angle initial_desired_angle = 0;
    // The desired angle of the crane, 0 = horizontal [rad]
    parameter Real Kp = 100;
    // Proportional gain
    parameter Real Ki = 10;
    // Integral gain
    parameter Real Kd = 100;
    // Derivative gain
    inner PlanarWorld planarWorld(defaultWidthFraction = 10) annotation(
      Placement(transformation(origin = {-76, 80}, extent = {{-10, -10}, {10, 10}})));
    // World inside which we're executing the simulation
    Fixed top_FP(r = {0, fixed_point_offset}) annotation(
      Placement(transformation(origin = {-72, 40}, extent = {{10, -10}, {-10, 10}})));
    // Top fixed point (top spring and control wire)
    Fixed middle_FP annotation(
      Placement(transformation(origin = {-74, 0}, extent = {{10, -10}, {-10, 10}})));
    // Middle fixed point (crane arm)
    Fixed bottom_FP(r = {0, -fixed_point_offset}) annotation(
      Placement(transformation(origin = {-60, -52}, extent = {{10, -10}, {-10, 10}})));
    // Bottom fixed point (bottom spring and control wire)
    // Create crane arm (we model it as 3 parts, so that we have 2 intermediate points along the arm where we can connect the springs and control wires)
    Joint crane_arm_join annotation(
      Placement(transformation(origin = {-44, 0}, extent = {{-10, -10}, {10, 10}})));
    FixedTranslation crane_arm1(r = {l_crane_arm/3, 0}, width = 0.1);
    // Object to represent the first part of the arm
    FixedTranslation crane_arm2(r = {l_crane_arm/3, 0}, width = 0.1);
    // Object to represent the second part of the arm
    FixedTranslation crane_arm3(r = {l_crane_arm/3, 0}, width = 0.1);
    // Object to represent the third part of the arm
    Body crane_arm_weight(m = m_crane, I = inertia);
    // Object to represent crane arm weight
    // Create a wire to hold the load (will be modeled as a rigid bar), has no weight
    FixedTranslation load_wire(r = {0, -l_load_wire}, width = 0.02);
    // Very small width to simulate thin wire
    LoadAttachment crane_wire_join;
    // Object to enable rotation of wire attached to the end of the crane arm
    // Create load on the crane
    Body load(m = m_load, I = inertia, sphereDiameter = 0.3);
    // Crane load (mass point)
    // Create spring that holds the crane arm from the top
    CraneSpring spring_top(k_spring = k_spring_top);
    // Create spring that holds the crane arm from the bottom
    CraneSpring spring_bottom(k_spring = k_spring_bottom);
    // Create force from top control wire
    Angle top_force_angle;
    // Angle at which the top force is acting on the crane arm [rad]
    PullWire wire_top annotation(
      Placement(transformation(origin = {-18, 48}, extent = {{-10, -10}, {10, 10}})));
    
    // Create force from bottom control wire
    Angle bottom_force_angle;
    // Angle at which the bottom force is acting on the crane arm [rad]
    PullWire wire_bottom annotation(
      Placement(transformation(origin = {2, -20}, extent = {{-10, -10}, {10, 10}})));
    Real crane_x;
    // X coordinate of the end of the crane arm
    Real crane_y;
    // Y coordinate of the end of the crane arm
    Angle crane_angle;
    // Angle of the crane (positive = above level, negative = below level) [rad]
    Force top_wire_force;
    // Force applied to the top wire
    Force bottom_wire_force;
    // Force applied to the bottom wire
    // Variables for PID controller
    Angle angle_error;
    Real integral_error(start = 0);
    Real derivative_error;
    Force control_force;
    Real desired_angle(start = initial_desired_angle);
  equation
// Create crane arm with the wire
    connect(middle_FP.frame, crane_arm_join.frame_a) annotation(
      Line(points = {{-64, 0}, {-54, 0}}, color = {95, 95, 95}));
// Attach join to fixed point
    connect(crane_arm_join.frame_b, crane_arm1.frame_a);
// Attach first part of the arm to join
    connect(crane_arm1.frame_b, crane_arm2.frame_a);
// Attach second part of the arm to the first
    connect(crane_arm2.frame_b, crane_arm3.frame_a);
// Attach third part the arm to the second
    connect(crane_arm3.frame_b, crane_arm_weight.frame_a);
// Attach weight to the end of the arm (end of 3rd part)
    connect(crane_arm3.frame_b, crane_wire_join.frame_a);
// Attach join for the wire to the end of the arm (end of 3rd part)
    connect(crane_wire_join.frame_b, load_wire.frame_a);
// Attach wire to the join at the end of the arm
// Attach load to the wire
    connect(load.frame_a, load_wire.frame_b);
// Create top spring
    connect(spring_top.frame_a, top_FP.frame);
// Attach spring to fixed point
    connect(crane_arm1.frame_b, spring_top.frame_b);
// Attach spring to crane arm
// Create bottom spring
    connect(spring_bottom.frame_a, bottom_FP.frame);
// Attach spring to fixed point
    connect(crane_arm1.frame_b, spring_bottom.frame_b);
    
  connect(top_FP.frame, wire_top.frame_a) annotation(
      Line(points = {{-62, 40}, {-28, 40}, {-28, 48}}, color = {95, 95, 95}));
  connect(bottom_FP.frame, wire_bottom.frame_a) annotation(
      Line(points = {{-50, -52}, {-8, -52}, {-8, -20}}, color = {95, 95, 95}));
    
// Attach spring to crane arm
// Determine crane location
    crane_x = crane_arm3.frame_b.x;
    crane_y = crane_arm3.frame_b.y;
    crane_angle = asin(crane_y/l_crane_arm);
// Determine the angle at which the force from the top wire is acting on the arm and then create it
    top_force_angle = asin(fixed_point_offset*sin(pi/2 + crane_angle)/sqrt(fixed_point_offset*fixed_point_offset + (l_crane_arm*2/3)*(l_crane_arm*2/3) - 2*(l_crane_arm*2/3)*fixed_point_offset*cos(pi/2 + crane_angle)));
    connect(crane_arm2.frame_b, wire_top.frame_b);
// Determine the angle at which the force from the bottom wire is acting on the arm and then create it
    bottom_force_angle = asin(fixed_point_offset*sin(pi/2 - crane_angle)/sqrt(fixed_point_offset*fixed_point_offset + (l_crane_arm*2/3)*(l_crane_arm*2/3) - 2*(l_crane_arm*2/3)*fixed_point_offset*cos(pi/2 - crane_angle)));
    connect(crane_arm2.frame_b, wire_bottom.frame_b);
// Apply forces so that the crane angle is as close as possible to the desired angle using a PID controller
    angle_error = desired_angle - crane_angle;
    der(integral_error) = angle_error;
    derivative_error = der(angle_error);
    control_force = Kp*angle_error + Ki*integral_error + Kd*derivative_error;
    top_wire_force = control_force/2;
    wire_top.wanted_force = {-sin(top_force_angle)*top_wire_force, cos(top_force_angle)*top_wire_force, 0};
// Distribute force to the top wire
    bottom_wire_force = -control_force/2;
    wire_bottom.wanted_force = {-sin(bottom_force_angle)*bottom_wire_force, -cos(bottom_force_angle)*bottom_wire_force, 0};
// Distribute force to the bottom wire
  end Crane;

  model Testbench1
    Crane sut;
  equation
  // Control desired angle over time
    if time < 20 then
      sut.desired_angle = 0;
    elseif time < 40 then
      sut.desired_angle = 10*Modelica.Constants.pi/180;
    elseif time < 60 then
      sut.desired_angle = 0;
    else
      sut.desired_angle = -10*Modelica.Constants.pi/180;
    end if;
    
    sut.crane_arm_join.state = FaultType.ok;
    sut.crane_wire_join.state = FaultType.ok;
    sut.wire_top.state = FaultType.ok;
    sut.wire_bottom.state = FaultType.ok;
    sut.spring_top.state = FaultType.ok;
    sut.spring_bottom.state = FaultType.ok;
    
    annotation(
      experiment(StartTime = 0, StopTime = 100, Tolerance = 1e-06, Interval = 0.01));
  end Testbench1;
  
  annotation(
    uses(PlanarMechanics(version = "1.6.0")));

end CraneFaultModel;
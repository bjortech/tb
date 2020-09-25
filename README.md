# tb
Generic Highlevel Autonomy Framework

# Freak introduksjon - del5!
Nå er vi komemt der det begynner å bli spennende. Så langt har vi satt opp simulering, robot, kartlegging i 3D, projisering til 2D for pathplanning og kontroll over systemets aktuatorer (servoer/bevegelige "ledd" (joints)).

I del#5 ser vi første tendenser til autonom atferd: Ved å beregne posisjonen litt frem i tid kan kollisjonsjekker kjøres direkte på to objekter, noe som er ganske rett frem så lenge man har begge i samme frame. (Noe det også er ganske rett frem å få til - takket være tf2_ros.)



# Automatisert installasjon av ROS && bygging/sourcing av denne branch:

https://github.com/bjortech/tb/tree/freak/tb_installscripts

# Byggeklosser fra eksterne parter som launches i eksempelet:

move_base:
http://wiki.ros.org/move_base

robot_state_publisher:
http://wiki.ros.org/robot_state_publisher

tf2:
http://wiki.ros.org/tf2/

map_server:
http://wiki.ros.org/map_server

universal_robot_description_file:
http://wiki.ros.org/urdf

# Eksempel video:

- kommer

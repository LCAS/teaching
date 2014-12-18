% locations of QR-Codes
qr( 1).
qr( 3).
qr( 4).
qr( 6).
qr( 7).
qr( 8).
qr( 9).
qr(10).
qr(11).

% possible contents of QR-Codes
content(-1).
content(0).
content(1).
content(2).
content(3).
content(4).
content(5).
content(6).

% QR-Codes that share content
opposite(4,7).
opposite(6,8).
opposite(9,11).
opposite(X,Y) :- opposite(Y,X).

% locations
location( 1).
location( 2).
location( 3).
location( 4).
location( 5).
location( 6).
location( 7).
location( 8).
location( 9).
location(10).
location(11).

% connections between locations
connection( 1, 2).
connection( 2, 3).
connection( 2, 4).
connection( 2, 5).
connection( 2, 6).
connection( 5, 7).
connection( 5, 8).
connection( 5, 9).
connection( 5,10).
connection( 9,11).
connection(X,Y) :- connection(Y,X).

% robots able to execute actions
robot(turtlebot_1).

% potential actions a robot is able to execute
potential(move(Location)) :- location(Location).
potential(scan).
potential(beep).

% initial situation of of the scenario
init(at(turtlebot_1,5)).
init(content( 1,1)).
init(content( 3,2)).
init(content(10,5)).
init(content(11,6)).

% request IDs, each request needs a unique ID
id(1..10).

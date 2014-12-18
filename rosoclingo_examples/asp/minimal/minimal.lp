#program base.

id(1..5).
maxtime(10).
time(1..M) :- maxtime(M).
robot(fulfill).
value(success).
value(failue).

#external _request(ID,Parameter,Time):id(ID),id(Parameter),time(Time).
#external _cancel(ID,Time):id(ID),time(Time).
#external _value(fulfill,Value,Time):value(Value),time(Time).

fulfilling(ID,Time) :- _action(fulfill,ID,Time), not _value(fulfill,failure,Time).

unfulfilled(ID,Time) :- _request(_,ID,Time), not _cancel(ID,Time).
unfulfilled(ID,Time) :- unfulfilled(ID,Time-1), time(Time), 1 > {fulfilling(ID,Time-1); _cancel(ID,Time)}.

possible(fulfill,ID,Time) :- unfulfilled(ID,Time).

{_action(Robot,Action,Time):possible(Robot,Action,Time)}1 :- time(Time), robot(Robot).
:- _action(_,_,T), time(T-1), not _action(_,_,T-1).

_satisfied(ID) :- _action(Robot,ID,Time), _value(Robot,success,Time).

goal(Time) :- time(Time), not unfulfilled(_,Time).
:- maxtime(M), not goal(M).

#show goal/1.
#show unfulfilled/2.
#show fulfilling/2.
#show _request/3.
#show _cancel/2.
#show _value/3.
#show _action/3.
#show _satisfied/1.

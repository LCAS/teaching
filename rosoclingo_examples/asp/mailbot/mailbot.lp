#const n=50. % horizon
#const o=5.  % offices
#const c=3.  % capacity
#const r=6.  % requests

time(1..n).
office(1..o).
id(1..r).

dir(-1; 1).
value(success;failure).

action(go(D))        :- dir(D).
action(pickup(O,P))  :- office(O;P).
action(deliver(O,P)) :- office(O;P).

holds(at(1),0).

#external _request(ID,bring(O,P),Time):id(ID),office(O),office(P),time(Time).
#external _cancel(ID,Time):id(ID),time(Time).
#external _value(mailbot,Parameter,Time) : value(Parameter),time(Time).

{ _action(mailbot,A,T) : action(A) } 1 :- time(T).
:- _action(_,_,T), time(T-1), not _action(_,_,T-1).
:- _action(_,_,T), goal(T-1).
:- _action(A,P,T), not poss(A,P,T).

poss(mailbot,go(D),T)		:- holds(at(O),T-1), office(O+D), dir(D), time(T).
poss(mailbot,pickup(O,P),T)	:- holds(at(O),T-1), { holds(holding(_,_),T-1) } c-1, received(request(O,P),T-1), time(T).
poss(mailbot,deliver(O,P),T)	:- holds(at(P),T-1), holds(holding(O,P),T-1), time(T).

holds(at(O+D),T) :- _action(mailbot,go(D),T), not _value(mailbot,failue,T), holds(at(O),T-1), office(O+D).
holds(at(O  ),T) :- not _action(mailbot,go(_),T), holds(at(O),T-1), time(T).
holds(at(O  ),T) :- _value(mailbot,failure,T), holds(at(O),T-1), time(T).

holds(holding(O,P),T) :- _action(mailbot,pickup(O,P),T), not _value(mailbot,failure,T).

holds(holding(O,P),T) :- holds(holding(O,P),T-1), not _action(mailbot,deliver(O,P),T), not received(cancel(O,P),T), time(T).
holds(holding(O,P),T) :- holds(holding(O,P),T-1), _action(mailbot,deliver(O,P),T), _value(mailbot,failure,T), not received(cancel(O,P),T), time(T).
holds(holding(O,O),T) :- holds(holding(O,P),T-1), received(cancel(O,P),T), time(T).

received(request(O,P),T) :- _request(_,bring(O,P),T), O!=P.
received(request(O,P),T) :- received(request(O,P),T-1), not _action(mailbot,pickup(O,P),T), not received(cancel(O,P),T), time(T).

received(cancel(O,P),T)  :- _cancel(ID,T), _request(ID,bring(O,P),_), O!=P.
received(cancel(O,P),T)  :- received(cancel(O,P),T-1),    holds(holding(O,P),T-1), time(T).
received(cancel(O,P),T)  :- received(cancel(O,P),T-1), received(request(O,P),T-1), time(T).

goal(T) :- not received(_,T), not holds(holding(_,_),T), time(T).

_satisfied(ID) :- _request(ID,bring(O,P),_), _action(mailbot,deliver(O,P),Time), _value(mailbot,success,Time).

:- not goal(n).

#show _action/3.
#show _request/3.
#show _cancel/2.
#show _value/3.
#show _satisfied/1.

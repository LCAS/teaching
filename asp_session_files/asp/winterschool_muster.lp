% ensuring that initial Fluents hold at time point 0
holds(Fluent,0) :- init(Fluent).

% describing the transition from the situation of one time point to the situation of its successor
#program transition(t).

% each robot may execute one potential action each time point
{_action(Robot,Action,t):potential(Action)}1 :- robot(Robot).
% it must be possible to execute the action at this time point
:- _action(Robot,Action,t), not possible(Robot,Action,t).

% identifying when actions are possible
possible(Robot,move(To),t)	:- holds(at(Robot,From),t-1), connection(From,To).
possible(Robot,beep,t)		:- holds(at(Robot,QR),t-1), holds(request(ID,Content),t-1), holds(content(QR,Content),t-1).

% identifying actions that were executed successfully
executes(Robot,Action,t) :- _action(Robot,Action,t), not _value(Robot,failure,t).

% identifying the effect of actions / external events
%holds(request(ID,Content),t)	:- _request(ID,moveto(Content),t), holds(content(QR,Content),t), not _cancel(ID,t).
holds(at(Robot,Location),t)	:- executes(Robot,move(Location),t).

% identifying fluents that ceases to exists in the next time point as a result of an action
abnormal(request(ID,Content),t)	:- holds(request(ID,Content),t-1), holds(at(Robot,QR),t), holds(content(QR,Content),t), executes(Robot,beep,t).
abnormal(at(Robot,Location),t)	:- holds(at(Robot,Location),t-1), executes(Robot,move(_),t).

% handling the frame problem, fluents stay the same unless something (abnormal) happened to them
holds(Fluent,t) :- holds(Fluent,t-1), not abnormal(Fluent,t).

% the request to move to a specific content as external event
#external _request(ID,moveto(Content),t)	: id(ID),content(Content).
% canceling the request with the id ID
#external _cancel(ID,t)				: id(ID).
% the return values of actions executed as external events
#external _value(Robot,success,t)		: robot(Robot).
#external _value(Robot,failure,t)		: robot(Robot).
#external _value(Robot,Content,t)		: robot(Robot), content(Content).

% identifying situations where there are no requests left unsolved
%goal(t) :- not holds(request(_,_),t).

% an external identifying the last time step of the task plan
#external horizon(t).
% the last time step of the task plan must be an situation with the goal achieved
:- not goal(t), horizon(t).

%%%%%%%%%%% Do your own work here

%%% cancel actions %%%
% receiving an external event canceling a request CEASES the need to fulfill the request
abnormal(request(ID,Content),t) :- holds(request(ID,Content),t-1), _cancel(ID,t).
% line 20 needed to be modified to incorporate for the situation that a request was canceled in the same time point is was issued.
holds(request(ID,Content),t)	:- _request(ID,moveto(Content),t), holds(content(QR,Content),t), not _cancel(ID,t).

%%% Indirect Knowledge %%%
% WHENEVER I know the content of a QR-Code THEN I know the content of it's counterpart
holds(content(QR2,Content),t) :- holds(content(QR1,Content),t), not holds(content(QR2,-1),t), opposite(QR1,QR2).

%%% scan action %%%
% possible: WHENEVER a robot is at a location AND the location has a QR-Code AND the content of the QR-Code was not known in the previous time point THEN it is possible for the robot to execute a scan action
possible(Robot,scan,t)		:- holds(at(Robot,QR),t-1), qr(QR), not holds(content(QR,_),t-1).

% effect: a robot successfully executing a scan action AND the action having return value AND the robot being at a location with a QR-Code CAUSES the QR-Code to have the content of the return value
holds(content(QR,Content),t)	:- executes(Robot,scan,t), _value(Robot,Content,t), holds(at(Robot,QR),t).

% WHEN a request is issued to go to a content AND no QR-Code with this content is known THEN the content needs to be searched for
holds(search(ID,Content),t) :- _request(ID,moveto(Content),t), not holds(content(_,Content),t), not _cancel(ID,t).

% canceling a request CEASES the need to search further for the content
abnormal(search(ID,Content),t) :- holds(search(ID,Content),t-1), _cancel(ID,t).

% executing a scanning action AND not jet receiving a result CEASES the need to search further for the content
abnormal(search(ID,Content),t) :- holds(search(ID,Content),t-1), _action(Robot,scan,t), not _value(Robot,_,t).

% executing a scanning action AND receiving the content searched for CEASES the need to search further fo the content
abnormal(search(ID,Content),t) :- holds(search(ID,Content),t-1), _action(Robot,scan,t), _value(Robot,Content,t).

% executing a scanning action AND receiving content searched for CAUSES the requests to be satisfiable
holds(request(ID,Content),t) :- holds(search(ID,Content),t-1), executes(Robot,scan,t), _value(Robot,Content,t).

% modify the goal identification in line 38 such that a goal situation is only achieved when no requests are outstanding AND no search needs to be performed
goal(t) :- not holds(request(_,_),t), not holds(search(_,_),t).

%%%%%%%%%%%

% a program part for ROSoClingo for committing the solver to actions executed
#program action_commit(ro,a,t).
:- not _action(ro,a,t).

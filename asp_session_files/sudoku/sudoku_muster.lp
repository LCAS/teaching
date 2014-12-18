value(1..S*S)  :- sub_size(S).

subsquare(X,Y,(X-1)/S*S+(Y-1)/S+1) :- value(X), value(Y), sub_size(S).

sudoku(X,Y,N) :- initial(X,Y,N).

1 { sudoku(X,Y,N) : value(N) } 1 :- value(X), value(Y).

:- { sudoku(X,Y,N) } != 1, value(X), value(N).
:- { sudoku(X,Y,N) } != 1, value(Y), value(N).
:- { sudoku(X,Y,N) : subsquare(X,Y,S) } != 1, value(N), value(S).

#show sudoku/3.

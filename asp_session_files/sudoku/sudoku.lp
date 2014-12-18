cell(1..N*N) :- sub_size(N).
{ sudoku(I,J,N) : cell(I), cell(J), cell(N) } .
:- sub_size(N), not N**4 { sudoku(I,J,_) } N**4 .
sudoku(I,J,X) :- initial(I,J,X).
:- sudoku(I,J,X), sudoku(I,J',Y), J!=J', X==Y.
:- sudoku(I,J,X), sudoku(I',J,Y), I!=I', X==Y.
subgrid(X,Y,G) :- cell(X), cell(Y), sub_size(S), G = (X-1)/S*S+(Y-1)/S+1.
:- sudoku(I,J,X), subgrid(I,J,G), sudoku(I',J',X'), subgrid(I',J',G'), I'!=I, J'!=J, G'==G, X==X'.

#show sudoku/3.
%show initial/3.

%debug
%{
#include <stdio.h>
#include <stdlib.h>
%}
%token NUMBER END
%nonassoc UMINUS
%{ 
  void yyerror(char *); 
  int yylex(void);
  void test();
  void test2();
  int matrix[4][3] = {
    {1, 1, 1},
    {1, 1, 1},
    {1, 1, 1},
    {1, 1, 1}
  };
%} 

%% 

prog: 
  'm' '[' expr ',' expr ']' '=' expr END { matrix[$3][$5] = $8; return 0;}
  ; 
expr: 
  term '+' expr { $$ = $1 + $3; }
  | term '-' expr { $$ = $1 - $3; } 
  | term { $$ = $1; }
  ; 
term:
  factor '*' term { $$ = $1 * $3; }
  | factor '/' term { $$ = $1 / $3; }
  | factor { $$ = $1; }
  | '-' factor %prec UMINUS { $$ = -$2; }
  ;
factor:
  NUMBER { $$ = $1; }
  | '(' expr ')' { $$ = $2; }
  | 'm' '[' expr ',' expr ']' { $$ = matrix[$3][$5]; }
  ;

%% 

void yyerror(char *s) { fprintf(stderr, "%s\n", s); } 

int main(void) { 
  // yydebug = 1;
  test();
  for (int i = 0; i < 4; i++) {
    printf("[ %d, %d, %d ]", matrix[i][0], matrix[i][1], matrix[i][2]);
  }
} 

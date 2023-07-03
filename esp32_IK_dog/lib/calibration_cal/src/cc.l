%option noyywrap
%option yylineno

%{

#include <stdlib.h>
#include "cc.h"

%}

%%
[0] { yylval = 0; return NUMBER; }
[123456789][[:digit:]]*	{ yylval = atoi(yytext); return NUMBER; }
[()=\[\]m,+\-*/]	{ return *yytext; }
\n	{ return END; }
[[:space:]]
%%
void test() {
    yy_scan_string("m[2,2]=100\n");
    yyparse();
}
%option noyywrap
%option yylineno
%{

#include <stdlib.h>
#include "cc.hpp"

%}

%%
[0] { yylval = 0; return NUMBER; }
[123456789][[:digit:]]*	{ yylval = atoi(yytext); return NUMBER; }
[()=\[\]m,+\-*/]	{ return *yytext; }
\n	{ return END; }
[[:space:]]
%%
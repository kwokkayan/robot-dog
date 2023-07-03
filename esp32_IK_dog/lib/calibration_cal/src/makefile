cc: cc.y cc.l
	bison -d cc.y -o cc.c
	flex -o lex.cc.c cc.l
	gcc -o cc cc.c lex.cc.c

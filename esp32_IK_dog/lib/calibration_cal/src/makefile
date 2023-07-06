cc: cc.y cc.l
	bison -d cc.y -o cc.cpp
	flex --header-file=lex.cc.hpp -o lex.cc.cpp cc.l

clean:
	rm cc.hpp cc.cpp lex.cc.cpp lex.cc.hpp
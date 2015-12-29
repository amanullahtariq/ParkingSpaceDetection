#pragma once
#pragma region Standard Library
#include <stdio.h>
#pragma endregion
class Train
{
public:
	struct model* model_;
	int flag_predict_probability;

public:
	Train(void);
	~Train(void);

	/*void exit_input_error(int line_num);
	void do_predict(FILE *input, FILE *output);
	char* readline(FILE *input);
	void ExecutePredict(int argc, char **argv);*/
};


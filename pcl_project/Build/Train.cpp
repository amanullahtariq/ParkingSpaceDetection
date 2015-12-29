#include "Train.h"

#pragma region standard library
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include "linear.h"
#pragma endregion


#pragma region Namespace
//using namespace cv;
using namespace std;

#pragma endregion


#pragma region lib linear parameters

struct model* model_;
int flag_predict_probability=0;
static char *line = NULL;
static int max_line_len;
static int (*info)(const char *fmt,...) = &printf;
int max_nr_attr = 64;
struct feature_node *x;

struct feature_node *x_space;
struct parameter param;
struct problem prob;
int flag_cross_validation;
int flag_find_C;
int flag_C_specified;
int flag_solver_specified;
int nr_fold;
double bias;
int print_null(const char *s,...) {return 0;}

#define Malloc(type,n) (type *)malloc((n)*sizeof(type))
#define INF HUGE_VAL

#pragma endregion

#pragma region Constructor

Train::Train(void)
{
	flag_predict_probability = 0;
}

#pragma endregion

#pragma region Destructor

Train::~Train(void)
{
}

#pragma endregion

#pragma region Custom method
void exit_input_error(int line_num)
{
	fprintf(stderr,"Wrong input format at line %d\n", line_num);
	exit(1);
}

static char* readline(FILE *input)
{
	int len;

	if(fgets(line,max_line_len,input) == NULL)
		return NULL;

	while(strrchr(line,'\n') == NULL)
	{
		max_line_len *= 2;
		line = (char *) realloc(line,max_line_len);
		len = (int) strlen(line);
		if(fgets(line+len,max_line_len-len,input) == NULL)
			break;
	}
	return line;
}

void do_predict(FILE *input, FILE *output)
{
	int correct = 0;
	int total = 0;
	double error = 0;
	double sump = 0, sumt = 0, sumpp = 0, sumtt = 0, sumpt = 0;

	int nr_class=get_nr_class(model_);
	double *prob_estimates=NULL;
	int j, n;
	int nr_feature=get_nr_feature(model_);
	if(model_->bias>=0)
		n=nr_feature+1;
	else
		n=nr_feature;

	if(flag_predict_probability)
	{
		int *labels;

		if(!check_probability_model(model_))
		{
			fprintf(stderr, "probability output is only supported for logistic regression\n");
			exit(1);
		}

		labels=(int *) malloc(nr_class*sizeof(int));
		get_labels(model_,labels);
		prob_estimates = (double *) malloc(nr_class*sizeof(double));
		fprintf(output,"labels");
		for(j=0;j<nr_class;j++)
			fprintf(output," %d",labels[j]);
		fprintf(output,"\n");
		free(labels);
	}

	max_line_len = 1024;
	line = (char *)malloc(max_line_len*sizeof(char));
	while(readline(input) != NULL)
	{
		int i = 0;
		double target_label, predict_label;
		char *idx, *val, *label, *endptr;
		int inst_max_index = 0; // strtol gives 0 if wrong format

		label = strtok(line," \t\n");
		if(label == NULL) // empty line
			exit_input_error(total+1);

		target_label = strtod(label,&endptr);
		if(endptr == label || *endptr != '\0')
			exit_input_error(total+1);

		while(1)
		{
			if(i>=max_nr_attr-2)	// need one more for index = -1
			{
				max_nr_attr *= 2;
				x = (struct feature_node *) realloc(x,max_nr_attr*sizeof(struct feature_node));
			}

			idx = strtok(NULL,":");
			val = strtok(NULL," \t");

			if(val == NULL)
				break;
			errno = 0;
			x[i].index = (int) strtol(idx,&endptr,10);
			if(endptr == idx || errno != 0 || *endptr != '\0' || x[i].index <= inst_max_index)
				exit_input_error(total+1);
			else
				inst_max_index = x[i].index;

			errno = 0;
			x[i].value = strtod(val,&endptr);
			if(endptr == val || errno != 0 || (*endptr != '\0' && !isspace(*endptr)))
				exit_input_error(total+1);

			// feature indices larger than those in training are not used
			if(x[i].index <= nr_feature)
				++i;
		}

		if(model_->bias>=0)
		{
			x[i].index = n;
			x[i].value = model_->bias;
			i++;
		}
		x[i].index = -1;

		if(flag_predict_probability)
		{
			int j;
			predict_label = predict_probability(model_,x,prob_estimates);
			fprintf(output,"%g",predict_label);
			for(j=0;j<model_->nr_class;j++)
				fprintf(output," %g",prob_estimates[j]);
			fprintf(output,"\n");
		}
		else
		{
			predict_label = predict(model_,x);
			fprintf(output,"%g\n",predict_label);
		}

		if(predict_label == target_label)
			++correct;
		error += (predict_label-target_label)*(predict_label-target_label);
		sump += predict_label;
		sumt += target_label;
		sumpp += predict_label*predict_label;
		sumtt += target_label*target_label;
		sumpt += predict_label*target_label;
		++total;
	}
	if(check_regression_model(model_))
	{
		info("Mean squared error = %g (regression)\n",error/total);
		info("Squared correlation coefficient = %g (regression)\n",
			((total*sumpt-sump*sumt)*(total*sumpt-sump*sumt))/
			((total*sumpp-sump*sump)*(total*sumtt-sumt*sumt))
			);
	}
	else
		info("Accuracy = %g%% (%d/%d)\n",(double) correct/total*100,correct,total);
	if(flag_predict_probability)
		free(prob_estimates);
}

void ExecutePredict(int argc, char **argv)
{
	char input_file_name[1024];
	char output_file_name[1024];
	char model_file_name[1024];
	FILE *input, *output;
	int i;
	strcpy( input_file_name,"C:\\Users\\amanullahtariq\\Documents\\Visual Studio 2010\\Projects\\pcl_project\\Data\\predict.txt");
	strcpy( output_file_name,"C:\\Users\\amanullahtariq\\Documents\\Visual Studio 2010\\Projects\\pcl_project\\Data\\output.out");
	strcpy( model_file_name,"C:\\Users\\amanullahtariq\\Documents\\Visual Studio 2010\\Projects\\pcl_project\\Data\\train.model");
	//flag_predict_probability = atoi(argv[i]);
	// parse options
	//for(i=1;i<argc;i++)
	//{
	//	if(argv[i][0] != '-') break;
	//	++i;
	//	switch(argv[i-1][1])
	//	{
	//		case 'b':
	//			flag_predict_probability = atoi(argv[i]);
	//			break;
	//		case 'q':
	//			info = &print_null;
	//			i--;
	//			break;
	//		default:
	//			fprintf(stderr,"unknown option: -%c\n", argv[i-1][1]);
	//			exit_with_help();
	//			break;
	//	}
	//}


	input = fopen(input_file_name,"r");
	if(input == NULL)
	{
		fprintf(stderr,"can't open input file %s\n",input_file_name);
		exit(1);
	}

	output = fopen(output_file_name,"w");
	if(output == NULL)
	{
		fprintf(stderr,"can't open output file %s\n",output_file_name);
		exit(1);
	}

	if((model_=load_model(model_file_name))==0)
	{
		fprintf(stderr,"can't open model file %s\n",model_file_name);
		exit(1);
	}

	x = (struct feature_node *) malloc(max_nr_attr*sizeof(struct feature_node));
	do_predict(input, output);
	free_and_destroy_model(&model_);
	free(line);
	free(x);
	fclose(input);
	fclose(output);
}

#pragma region train
void print_null(const char *s) {}

void exit_with_help()
{
	printf(
	"usage: train [options] training_set_file [model_file]\n"
	"options:\n"
	"-s type : set type of solver (default 1)\n"
	"  for multi-class classification\n"
	"	 0 -- l2-regularized logistic regression (primal)\n"
	"	 1 -- l2-regularized l2-loss support vector classification (dual)\n"
	"	 2 -- l2-regularized l2-loss support vector classification (primal)\n"
	"	 3 -- l2-regularized l1-loss support vector classification (dual)\n"
	"	 4 -- support vector classification by crammer and singer\n"
	"	 5 -- l1-regularized l2-loss support vector classification\n"
	"	 6 -- l1-regularized logistic regression\n"
	"	 7 -- l2-regularized logistic regression (dual)\n"
	"  for regression\n"
	"	11 -- l2-regularized l2-loss support vector regression (primal)\n"
	"	12 -- l2-regularized l2-loss support vector regression (dual)\n"
	"	13 -- l2-regularized l1-loss support vector regression (dual)\n"
	"-c cost : set the parameter c (default 1)\n"
	"-p epsilon : set the epsilon in loss function of svr (default 0.1)\n"
	"-e epsilon : set tolerance of termination criterion\n"
	"	-s 0 and 2\n"
	"		|f'(w)|_2 <= eps*min(pos,neg)/l*|f'(w0)|_2,\n"
	"		where f is the primal function and pos/neg are # of\n"
	"		positive/negative data (default 0.01)\n"
	"	-s 11\n"
	"		|f'(w)|_2 <= eps*|f'(w0)|_2 (default 0.001)\n"
	"	-s 1, 3, 4, and 7\n"
	"		dual maximal violation <= eps; similar to libsvm (default 0.1)\n"
	"	-s 5 and 6\n"
	"		|f'(w)|_1 <= eps*min(pos,neg)/l*|f'(w0)|_1,\n"
	"		where f is the primal function (default 0.01)\n"
	"	-s 12 and 13\n"
	"		|f'(alpha)|_1 <= eps |f'(alpha0)|,\n"
	"		where f is the dual function (default 0.1)\n"
	"-b bias : if bias >= 0, instance x becomes [x; bias]; if < 0, no bias term added (default -1)\n"
	"-wi weight: weights adjust the parameter c of different classes (see readme for details)\n"
	"-v n: n-fold cross validation mode\n"
	"-c : find parameter c (only for -s 0 and 2)\n"
	"-q : quiet mode (no outputs)\n"
	);
	exit(1);
}

//void exit_input_error(int line_num)
//{
//	fprintf(stderr,"wrong input format at line %d\n", line_num);
//	exit(1);
//}

//static char *line = null;
//static int max_line_len;

//static char* readline(file *input)
//{
//	int len;
//
//	if(fgets(line,max_line_len,input) == null)
//		return null;
//
//	while(strrchr(line,'\n') == null)
//	{
//		max_line_len *= 2;
//		line = (char *) realloc(line,max_line_len);
//		len = (int) strlen(line);
//		if(fgets(line+len,max_line_len-len,input) == null)
//			break;
//	}
//	return line;
//}

void do_find_parameter_c()
{
	double start_c, best_c, best_rate;
	double max_c = 1024;
	if (flag_c_specified)
		start_c = param.C;
	else
		start_c = -1.0;
	printf("doing parameter search with %d-fold cross validation.\n", nr_fold);
	find_parameter_c(&prob, &param, nr_fold, start_c, max_c, &best_c, &best_rate);
	printf("best c = %lf  cv accuracy = %g%%\n", best_c, 100.0*best_rate);
}

void do_cross_validation()
{
	int i;
	int total_correct = 0;
	double total_error = 0;
	double sumv = 0, sumy = 0, sumvv = 0, sumyy = 0, sumvy = 0;
	double *target = malloc(double, prob.l);

	cross_validation(&prob,&param,nr_fold,target);
	if(param.solver_type == l2r_l2loss_svr ||
	   param.solver_type == l2r_l1loss_svr_dual ||
	   param.solver_type == l2r_l2loss_svr_dual)
	{
		for(i=0;i<prob.l;i++)
		{
			double y = prob.y[i];
			double v = target[i];
			total_error += (v-y)*(v-y);
			sumv += v;
			sumy += y;
			sumvv += v*v;
			sumyy += y*y;
			sumvy += v*y;
		}
		printf("cross validation mean squared error = %g\n",total_error/prob.l);
		printf("cross validation squared correlation coefficient = %g\n",
				((prob.l*sumvy-sumv*sumy)*(prob.l*sumvy-sumv*sumy))/
				((prob.l*sumvv-sumv*sumv)*(prob.l*sumyy-sumy*sumy))
			  );
	}
	else
	{
		for(i=0;i<prob.l;i++)
			if(target[i] == prob.y[i])
				++total_correct;
		printf("cross validation accuracy = %g%%\n",100.0*total_correct/prob.l);
	}

	free(target);
}

void parse_command_line(int argc, char **argv, char *input_file_name, char *model_file_name)
{
	int i;
	void (*print_func)(const char*) = null;	// default printing to stdout

	// default values
	param.solver_type = l2r_l2loss_svc_dual;
	param.c = 1;
	param.eps = inf; // see setting below
	param.p = 0.1;
	param.nr_weight = 0;
	param.weight_label = null;
	param.weight = null;
	param.init_sol = null;
	flag_cross_validation = 0;
	flag_c_specified = 0;
	flag_solver_specified = 0;
	flag_find_c = 0;
	bias = -1;

	// parse options
	for(i=1;i<argc;i++)
	{
		if(argv[i][0] != '-') break;
		if(++i>=argc)
			exit_with_help();
		switch(argv[i-1][1])
		{
			case 's':
				param.solver_type = atoi(argv[i]);
				flag_solver_specified = 1;
				break;

			case 'c':
				param.c = atof(argv[i]);
				flag_c_specified = 1;
				break;

			case 'p':
				param.p = atof(argv[i]);
				break;

			case 'e':
				param.eps = atof(argv[i]);
				break;

			case 'b':
				bias = atof(argv[i]);
				break;

			case 'w':
				++param.nr_weight;
				param.weight_label = (int *) realloc(param.weight_label,sizeof(int)*param.nr_weight);
				param.weight = (double *) realloc(param.weight,sizeof(double)*param.nr_weight);
				param.weight_label[param.nr_weight-1] = atoi(&argv[i-1][2]);
				param.weight[param.nr_weight-1] = atof(argv[i]);
				break;

			case 'v':
				flag_cross_validation = 1;
				nr_fold = atoi(argv[i]);
				if(nr_fold < 2)
				{
					fprintf(stderr,"n-fold cross validation: n must >= 2\n");
					exit_with_help();
				}
				break;

			case 'q':
				print_func = &print_null;
				i--;
				break;

			case 'c':
				flag_find_c = 1;
				i--;
				break;

			default:
				fprintf(stderr,"unknown option: -%c\n", argv[i-1][1]);
				exit_with_help();
				break;
		}
	}

	set_print_string_function(print_func);
	if(input_file_name == null)
	{
		// determine filenames
		if(i>=argc)
			exit_with_help();

		strcpy(input_file_name, argv[i]);

		if(i<argc-1)
			strcpy(model_file_name,argv[i+1]);
		else
		{
			char *p = strrchr(argv[i],'/');
			if(p==null)
				p = argv[i];
			else
				++p;
			sprintf(model_file_name,"%s.model",input_file_name);
		}
	}
	

	// default solver for parameter selection is l2r_l2loss_svc
	if(flag_find_c)
	{
		if(!flag_cross_validation)
			nr_fold = 5;
		if(!flag_solver_specified)
		{
			fprintf(stderr, "solver not specified. using -s 2\n");
			param.solver_type = l2r_l2loss_svc;
		}
		else if(param.solver_type != l2r_lr && param.solver_type != l2r_l2loss_svc)
		{
			fprintf(stderr, "warm-start parameter search only available for -s 0 and -s 2\n");
			exit_with_help();
		}
	}

	if(param.eps == inf)
	{
		switch(param.solver_type)
		{
			case l2r_lr:
			case l2r_l2loss_svc:
				param.eps = 0.01;
				break;
			case l2r_l2loss_svr:
				param.eps = 0.001;
				break;
			case l2r_l2loss_svc_dual:
			case l2r_l1loss_svc_dual:
			case mcsvm_cs:
			case l2r_lr_dual:
				param.eps = 0.1;
				break;
			case l1r_l2loss_svc:
			case l1r_lr:
				param.eps = 0.01;
				break;
			case l2r_l1loss_svr_dual:
			case l2r_l2loss_svr_dual:
				param.eps = 0.1;
				break;
		}
	}
}

// read in a problem (in libsvm format)
void read_problem(const char *filename)
{
	int max_index, inst_max_index, i;
	size_t elements, j;
	file *fp = fopen(filename,"r");
	char *endptr;
	char *idx, *val, *label;

	if(fp == null)
	{
		fprintf(stderr,"can't open input file %s\n",filename);
		exit(1);
	}

	prob.l = 0;
	elements = 0;
	max_line_len = 1024;
	line = malloc(char,max_line_len);
	while(readline(fp)!=null)
	{
		char *p = strtok(line," \t"); // label

		// features
		while(1)
		{
			p = strtok(null," \t");
			if(p == null || *p == '\n') // check '\n' as ' ' may be after the last feature
				break;
			elements++;
		}
		elements++; // for bias term
		prob.l++;
	}
	rewind(fp);

	prob.bias=bias;

	prob.y = malloc(double,prob.l);
	prob.x = malloc(struct feature_node *,prob.l);
	x_space = malloc(struct feature_node,elements+prob.l);

	max_index = 0;
	j=0;
	for(i=0;i<prob.l;i++)
	{
		inst_max_index = 0; // strtol gives 0 if wrong format
		readline(fp);
		prob.x[i] = &x_space[j];
		label = strtok(line," \t\n");
		if(label == null) // empty line
			exit_input_error(i+1);

		prob.y[i] = strtod(label,&endptr);
		if(endptr == label || *endptr != '\0')
			exit_input_error(i+1);

		while(1)
		{
			idx = strtok(null,":");
			val = strtok(null," \t");

			if(val == null)
				break;

			errno = 0;
			x_space[j].index = (int) strtol(idx,&endptr,10);
			if(endptr == idx || errno != 0 || *endptr != '\0' || x_space[j].index <= inst_max_index)
				exit_input_error(i+1);
			else
				inst_max_index = x_space[j].index;

			errno = 0;
			x_space[j].value = strtod(val,&endptr);
			if(endptr == val || errno != 0 || (*endptr != '\0' && !isspace(*endptr)))
				exit_input_error(i+1);

			++j;
		}

		if(inst_max_index > max_index)
			max_index = inst_max_index;

		if(prob.bias >= 0)
			x_space[j++].value = prob.bias;

		x_space[j++].index = -1;
	}

	if(prob.bias >= 0)
	{
		prob.n=max_index+1;
		for(i=1;i<prob.l;i++)
			(prob.x[i]-2)->index = prob.n;
		x_space[j-2].index = prob.n;
	}
	else
		prob.n=max_index;

	fclose(fp);
}

void executetrain(int argc, char **argv)
{
	char input_file_name[1024];
	char model_file_name[1024];
	const char *error_msg;
	char file_name[1024];
	strcpy( file_name,"c:\\users\\amanullahtariq\\documents\\visual studio 2012\\projects\\pcl_project\\data\\train");
	sprintf(input_file_name,"%s.txt",file_name);
	sprintf(model_file_name,"%s.model",file_name);
	//input_file_name = 'c:\liblinear\heart_scale';

	parse_command_line(argc, argv, input_file_name, model_file_name);
	read_problem(input_file_name);
	error_msg = check_parameter(&prob,&param);

	if(error_msg)
	{
		fprintf(stderr,"error: %s\n",error_msg);
		exit(1);
	}

	if (flag_find_c)
	{
		do_find_parameter_c();
	}
	else if(flag_cross_validation)
	{
		do_cross_validation();
	}
	else
	{
		model_=train(&prob, &param);
		if(save_model(model_file_name, model_))
		{
			fprintf(stderr,"can't save model to file %s\n",model_file_name);
			exit(1);
		}
		free_and_destroy_model(&model_);
	}
	destroy_param(&param);
	free(prob.y);
	free(prob.x);
	free(x_space);
	free(line);

	//return 0;
}

#pragma endregion

#pragma endregion

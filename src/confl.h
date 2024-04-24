/*  C programming reading and writing configuration files implementation
*
*
*	Each line is a record with the format as follows:
*
*			key1 = value1 # comments
*			key2 = value2 # comments
*			...
*
*	Comments after the '#' character will be ignored.
*
*	Leading and trailing whitespace of the key and value will be trimmed,
*	but there can be spaces in between.
*
*	Author: Liu Hengzhen
*	Date: April 22nd, 2024
*/

#ifndef _C_CONFIG_H_
#define _C_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Based on Linked List */
typedef struct config {
	char *name;
	char *value;
	struct config *next;
} config;

typedef struct configl {
	char *fn;
	struct config *confs;
	struct config *last;
} config_t;

config_t   *confRead	 (const char *file_name);
int         confSet		 (config_t *one, const char *key, const char *val);
void        confDel		 (config_t *one, const char *key);
int			confWrite	 (config_t *one);
void		confFree	 (config_t *one);
const char *confGet		 (const config_t *one, const char *key);
short		confGetInt16 (const config_t *one, const char *key);
int			confGetInt32 (const config_t *one, const char *key);
long long	confGetInt64 (const config_t *one, const char *key);
float		confGetFloat (const config_t *one, const char *key);
double		confGetDouble(const config_t *one, const char *key);

#ifdef __cplusplus
}
#endif

#endif

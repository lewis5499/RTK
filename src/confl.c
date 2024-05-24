#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#ifndef _UNISTD_H
#define _UNISTD_H
#include <io.h>
#include <process.h>
#else /* UNIX */
#include <unistd.h>
#endif /* _UNISTD_H */

#if defined(_MSC_VER)
#include <BaseTsd.h>
typedef SSIZE_T ssize_t;
#endif

#include "confl.h"

#pragma warning(disable:4996)

static ssize_t getline(char **buf, size_t *n, FILE *fp) {
	char c;
	int pos = 0;
	size_t maxlen = *n;
	char *buf_ptr = *buf;

	if (buf_ptr == NULL || maxlen == 0) {
		maxlen = 128;
		if ((buf_ptr = malloc(maxlen)) == NULL)
			return -1;
	}

	do {
		c = fgetc(fp);
		buf_ptr[pos++] = c;

		if (pos >= maxlen) {
			*buf = buf_ptr;
			buf_ptr = realloc(buf_ptr, maxlen *= 2);

			if (buf_ptr == NULL) {
				(*buf)[pos - 1] = '\0';
				return -1;
			}
		}
		if (c == EOF)
			return -1;

	} while (c != '\n');

	buf_ptr[pos] = '\0';
	*buf = buf_ptr;
	*n = maxlen;

	return pos;
}

static void freeConfigOne(struct config **one)
{
	struct config *tmp = *one;

	if (!tmp) {
		return;
	}
	if (tmp->name) {
		free(tmp->name);
		tmp->name = NULL;
	}
	if (tmp->value) {
		free(tmp->value);
		tmp->value = NULL;
	}

	*one = tmp->next;
	free(tmp);

	return;
}

static struct config **findNode(const config_t *one, const char *key)
{
	if (!one) {
		return NULL;
	}
	if (!one->confs) {
		return NULL;
	}
	if (!key) {
		return NULL;
	}

	if (!strcmp(one->confs->name, key)) {
		return (struct config **)&one->confs;
	}

	for (struct config *tmp = one->confs; tmp; tmp = tmp->next) {
		if (tmp->next && !strcmp(tmp->next->name, key)) {
			return (struct config **)&tmp->next;
		}
	}

	return NULL;
}

struct config *newConfig(const char *key, const char *val)
{
	size_t keyLen = strlen(key) + 1;
	size_t valLen = strlen(val) + 1;
	if (keyLen <= 1 || valLen <= 1) {
		return NULL;
	}

	struct config *one = malloc(sizeof(struct config));
	if (one) {
		one->next = NULL;

		one->name = (char *)malloc(keyLen);
		if (!one->name) {
			free(one);
			return NULL;
		}
		strcpy(one->name, key);

		one->value = (char *)malloc(valLen);
		if (!one->value) {
			free(one->name);
			free(one);
			return NULL;
		}
		strcpy(one->value, val);
	}

	return one;
}

config_t *newConfigl(const char *file_name, size_t len)
{
	config_t *one = (config_t *)malloc(sizeof(struct configl));
	if (!one) {
		return NULL;
	}

	one->fn = malloc(len + 1);
	if (!one->fn) {
		free(one);
		return NULL;
	}
	strcpy(one->fn, file_name);
	one->confs = NULL;
	one->last = NULL;

	return one;
}

config_t *confRead(const char *file_name)
{
	char *line = NULL;
	size_t len = 0;
	ssize_t read;

	size_t palen = strlen(file_name);
	if (palen <= 0) {
		return NULL;
	}

	FILE *fp = fopen(file_name, "r");
	if (NULL == fp) {
		return NULL;
	}

	config_t *one = newConfigl(file_name, palen);
	if (!one) {
		fclose(fp);
		return NULL;
	}

	while ((read = getline(&line, &len, fp)) != -1) {
		char *tmp = NULL, *space = NULL;
		char *key = NULL, *val = NULL;
		unsigned char stat = 0; // State machine

		for (tmp = line; *tmp; ++tmp) {
			if ('#' == *tmp || 0x0a == *tmp || 0x0d == *tmp) {
				if (space) {
					*space = 0;
				}
				else {
					*tmp = 0;
				}
				break;
			}
			else if (isspace(*tmp)) {
				if (1 == stat) {
					stat = 2;
					space = tmp;
				}
				else if (4 == stat) {
					stat = 5;
					space = tmp;
				}
				continue;
			}
			else if ('=' == *tmp) {
				stat = 3;
				if (space) {
					*space = 0;
					space = NULL;
				}
				else {
					*tmp = 0;
				}
				continue;
			}

			if (0 == stat) {
				stat = 1;
				key = tmp;
				continue;
			}
			if (3 == stat) {
				stat = 4;
				val = tmp;
				continue;
			}
			if (3 > stat) {
				stat = 1;
				space = NULL;
			}
			else if (5 == stat) {
				stat = 4;
				space = NULL;
			}
		}

		// New configuration items
		if (key && val) {
			confSet(one, key, val);
		}
	}

	free(line);
	fclose(fp);

	return one;
}

int confSet(config_t *one, const char *key, const char *val)
{
	if (!one || !key) {
		return -1; // Invalid input
	}

	struct config *exist = NULL;

	for (struct config *tmp = one->confs; tmp; tmp = tmp->next) {
		if (strcmp(tmp->name, key) == 0) {
			exist = tmp;
			break;
		}
	}

	if (exist) {
		size_t nl = strlen(val);
		char *nval = (char *)malloc(nl + 1);

		if (!nval) {
			return -2; // Memory allocation error
		}

		strcpy(nval, val);
		free(exist->value);
		exist->value = nval;

	}
	else {
		struct config *node = newConfig(key, val);
		if (!node) {
			return -3; // Node creation error
		}

		if (one->last) {
			one->last->next = node;
			one->last = node;
		}
		else {
			one->confs = node;
			one->last = node;
		}
	}

	return 0; // Success
}

void confDel(config_t *one, const char *key)
{
	struct config **tmp = findNode(one, key);
	if (tmp) {
		freeConfigOne(tmp);
	}

	return;
}

int confWrite(config_t *one)
{
	if (!one) {
		return -1;
	}
	if (!one->fn) {
		return -1;
	}

	FILE *fp = fopen(one->fn, "w");
	if (NULL == fp) {
		return -1;
	}

	for (struct config *tmp = one->confs; tmp; tmp = tmp->next) {
		if (tmp->name && tmp->value) {
			fprintf(fp, "%s = %s\n", tmp->name, tmp->value);
		}
	}

	fclose(fp);

	return 0;
}

void confFree(config_t *one)
{
	if (!one) {
		return;
	}
	if (one->fn) {
		free(one->fn);
		one->fn = NULL;
	}
	while (one->confs) {
		freeConfigOne(&one->confs);
	}
	free(one);
	one = NULL;

	return;
}

const char *confGet(const config_t *one, const char *key)
{
	struct config **tmp = findNode(one, key);
	if (!tmp) {
		return NULL;
	}

	return (const char *)(*tmp)->value;
}

short confGetInt16(const config_t *one, const char *key)
{
	return (short)atoi(confGet(one, key));
}

int confGetInt32(const config_t *one, const char *key)
{
	return atoi(confGet(one,key));
}

long long confGetInt64(const config_t *one, const char *key)
{
	return atoll(confGet(one, key));
}

float confGetFloat(const config_t *one, const char *key)
{
	return (float)atof(confGet(one, key));
}

double confGetDouble(const config_t *one, const char *key)
{
	return atof(confGet(one, key));
}
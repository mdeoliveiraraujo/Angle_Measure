/*
 * file_handling.h
 *
 *  Created on: 26-Jul-2019
 *      Author: arunr
 */

#ifndef FILE_HANDLING_H_
#define FILE_HANDLING_H_

#include "fatfs.h"
#include "string.h"
#include "stdio.h"

/* following are the commands that you can use
 * "ls " : lists all the directories and files in the given path
 * "mkdir " : creates a directory in the given path
 * "mkfil " : creates a file in the given path
 * "read " : reads the content of the given file and send them to uart
 * "write " : write the data to the given file
 * "rm " : removes a file or a directory. Directory can only be removed, if it is empty.
 * "update " : updates the content of the given file
 * "checkfile " : gives the details of the file
 * "checksd " : get free space of the sd card
 */
#define BUFFER_SIZE 512
#define ROW_SIZE 4
#define PATH_SIZE 32

extern char buffer[BUFFER_SIZE];  // to store strings..
extern char buffer_SD[BUFFER_SIZE*ROW_SIZE];
extern char path[PATH_SIZE];  // buffer to store path

/* get the size of the buffer, until the buffer is full*/
int bufsize (char *buf);

/*clears the buffer*/
void clear_buffer (void);

/*clears the buffer*/
void clear_buffer_SD (void);

/* clears the path buffer*/
void clear_path (void);

/* send the data to the uart*/
void send_uart (char *string);
void send_uart1 (char *string);

/* counts the length of the command
 * it checks for the space ' ' char. so make sure you give space after the command
 */
int cmdlength (char *str);

/* copies the path from the buffer to the pathbuffer*/
void get_path (void);

/* mounts the sd card*/
void mount_sd (void);

/* unmounts the sd card*/
void unmount_sd (void);

/* Start node to be scanned (***also used as work area***) */
FRESULT scan_files (char* pat);

/* write the data to the file
 * @ name : is the path to the file*/
void write_file (char *name);
void write_file_mod (char *name, char *buffer_adc);
void write_file_byte (char *name);

/* read data from the file
 * @ name : is the path to the file*/
void read_file (char *name);

/* creates the file, if it does not exists
 * @ name : is the path to the file*/
void create_file (char *name);
void create_file_mod(uint8_t BASE_NAME_SIZE, char *name);

void close_file(char *name);

/* Removes the file from the sd card
 * @ name : is the path to the file*/
void remove_file (char *name);

/* creates a directory
 * @ name: is the path to the directory
 */
void create_dir (char *name);

/* checks the free space in the sd card*/
void check_sd (void);

/* checks the details of the file
 * @ name : is the path to the file
 */
void check_file (char *name);

/* updates the file. write pointer is set to the end of the file
 * @ name : is the path to the file
 */
void update_file (char *name);




#endif /* FILE_HANDLING_H_ */

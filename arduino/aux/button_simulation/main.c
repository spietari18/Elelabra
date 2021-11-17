#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <time.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#include <termios.h>

#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/syscall.h>
#include <sys/sysmacros.h>

#include <linux/input.h>

/* how many keys are we using */
#define NUM_KEYS 3

/* how many events to read at once */
#define MAX_EVTS 16

/* if this many events are received
 * without KEY_ events in between
 * a warning is printed.
 */
#define EVT_WARN 128

/* if this many events are received
 * without KEY_ events in between
 * an error is raised
 */
#define EVT_ERROR 256

#define PRINT(FMT, ...) fprintf(stderr, ("[%lu] " FMT "\n"), syscall(SYS_gettid), ##__VA_ARGS__)
#define CHECK(CND, LABEL, ERRNO, FMT, ...) \
	do {\
		if ((CND)) { \
			if ((ERRNO)) { \
				PRINT(FMT ": %s", ##__VA_ARGS__, strerror(errno)); \
			} else { \
				PRINT(FMT, ##__VA_ARGS__); \
			} \
			goto LABEL; \
		} \
	} while (0) \


/* string versions of the key codes in
 * linux/input-event-codes.h, idea stolen
 * shamelessly from evtest
 */
static const char *const keys[KEY_CNT] = {
	[0 ... KEY_MAX] = NULL,
#define X(KEY) [KEY] = #KEY,
#include "keys.h"
#undef X
};

struct keyset {
	unsigned int    n; /* number of keys */
	unsigned short *c; /* n key codes */
	unsigned short *k; /* n key states */

	/* this is where our data is stored */
	unsigned short data[];
};

#define KEYSET(NAME, COUNT) \
	struct keyset NAME = {COUNT, NULL, NULL, \
		{[0 ... 2*sizeof(unsigned short)*COUNT] = 0}}; \
	NAME.c = (void *)&NAME + sizeof(NAME); \
	NAME.k = (void *)&NAME.c + sizeof(NAME.c)*COUNT \

struct event_listener_arg {
	struct keyset *set;
	int *flag;
	int fd;
};

/* simple linear search */
static int find_code(const char *str)
{
	for (size_t i = 0; i < KEY_CNT; i++)
		if (keys[i] && !strcmp(str, keys[i]))
			return i;
	return -1;
}

int loop = 1;

void on_signal(int sig)
{
	loop = 0;
}

pthread_t main_thread;
pthread_t child_thread;

static void *event_listener(struct event_listener_arg *arg)
{
	struct input_event ev[MAX_EVTS];

	void *ret = (void *)1;

	/* non KEY_ events in between KEY_ events */
	size_t nk_events = 0;

	/* Apparently it's too much for pthread_t to be
	 * boolean false if it's uninitialized. It's also
	 * apparently too much to add error checking in 
	 * libpthread to detect if the pthread_t that got
	 * passed to a pthread_ function was uninitialized.
	 */
	*arg->flag = 1;

	PRINT("Event listener thread.");

	/* block everything other than SIGUSR1 */
	sigset_t my_sigs;
	(void) sigfillset(&my_sigs);

	(void) sigdelset(&my_sigs, SIGUSR1);

	/* POSIX says these need to be enabled. */
	(void) sigdelset(&my_sigs, SIGSEGV);
	(void) sigdelset(&my_sigs, SIGFPE);
	(void) sigdelset(&my_sigs, SIGILL);

	CHECK(pthread_sigmask(SIG_BLOCK, &my_sigs, NULL),
		error, 1, "pthread_sigmask()");

	/* install signal handler */
	struct sigaction sa = {0};
	sa.sa_handler = on_signal;
	CHECK(sigaction(SIGUSR1, &sa, NULL), error, 1, "sigaction()");

	PRINT("Event listener loop started.");

	while (loop)
	{
		/* reset before call */
		errno = 0;

		/* /dev/input/ devices always return whole event structs */
		size_t count = read(arg->fd, ev,
			MAX_EVTS*sizeof(*ev)) / sizeof(*ev);

		/* EINTR likely means we got the interrupt signal */
		if (errno == EINTR) {
			/* SIGUSR1 handler unsets loop */
			if (!loop)
				break;
		/* other errors are fatal */
		} else CHECK(errno, exit, 1, "read()");

		/* process all events */
		for (size_t i = 0; i < count; i++)
		{
			/* warning and error */
			if (nk_events == EVT_WARN) {
				PRINT("WARNING: Not receiving "
					"enough keyboard events.");
			} else if (nk_events == EVT_ERROR) {
				PRINT("ERROR: Not receiving "
					"enough keyboard events.");
				goto error;
			}

			/* ignore non-key events but increment counter */
			if (ev[i].type != EV_KEY) {
				nk_events++;
				continue;
			/* ignore everything other than key up and down */
			} else if (ev[i].value > 1) {
				continue;
			}

			for (size_t j = 0; j < arg->set->n; j++)
			{
				/* if we're capturing this key, update state */
				if (arg->set->c[j] == ev[i].code) {
					arg->set->k[j] = ev[i].value;
					break;
				}
			}

			nk_events = 0;
		}
	}

	PRINT("Event listener loop interrupted.");

	ret = (void *)0;

	goto exit;
error:
	/* signal main thread that something is wrong */
	pthread_kill(main_thread, SIGUSR1);

	/* handle cleanup here */
exit:
	PRINT("Event listener thread exit.");

	return ret;
}

extern void code(unsigned long);

int main(int argc, char *argv[])
{
	static KEYSET(set, NUM_KEYS);

	struct termios attr;

	int terminal_modified_flag = 0, child_exists_flag = 0, ret = 1;

	main_thread = pthread_self();
	PRINT("Main thread.");

	/* Check that we have a correct number of args. */
	if (argc != (2 + set.n)) {
		(void) fprintf(stderr,
			"Usage: %s DEVICE", argv[0]);
		for (size_t i = 0; i < set.n; i++)
			(void) fprintf(stderr, " KEY%zu", i);
		(void) fputc('\n', stderr);
		goto exit;
	}

	/* block everything other than SIGUSR1, SIGTERM and SIGINT */
	sigset_t my_sigs;
	(void) sigfillset(&my_sigs);

	(void) sigdelset(&my_sigs, SIGUSR1);
	(void) sigdelset(&my_sigs, SIGTERM);
	(void) sigdelset(&my_sigs, SIGINT);

	/* POSIX says these need to be enabled. */
	(void) sigdelset(&my_sigs, SIGSEGV);
	(void) sigdelset(&my_sigs, SIGFPE);
	(void) sigdelset(&my_sigs, SIGILL);

	CHECK(pthread_sigmask(SIG_BLOCK, &my_sigs, NULL),
		exit, 1, "pthread_sigmask()");

	/* install signal handlers */
	struct sigaction sa = {0};
	sa.sa_handler = on_signal;
	CHECK(sigaction(SIGUSR1, &sa, NULL), exit, 1, "sigaction()");
	CHECK(sigaction(SIGTERM, &sa, NULL), exit, 1, "sigaction()");
	CHECK(sigaction(SIGINT, &sa, NULL), exit, 1, "sigaction()");

	/* State the file we were given. */
	struct stat st;
	CHECK(stat(argv[1], &st), exit, 1, "stat()");

	/* Make sure this is a character device */

	CHECK(!(S_ISCHR(st.st_mode) && (major(st.st_rdev) == 13)),
		exit, 0, "%s is not an input device.", argv[1]);

	/* Open our device. */
	int fd;
	CHECK((fd = open(argv[1], O_RDONLY)) == -1, exit, 1, "open()");

	/* Read keys */
	for (size_t i = 0; i < set.n; i++)
	{
		size_t j;	
		int code;

		/* Check that the code is valid */
		CHECK((code = find_code(argv[2 + i])) == -1,
			exit, 0, "%s is not a valid keycode.", argv[2 + i]);

		/* Check that the code isn't a duplicate */
		for (size_t j = 0; j < i; j++)
			CHECK(set.c[j] == code, exit, 0,
				"%s is specified multiple times.", keys[code]);

		set.c[i] = code;
	}

	/* Get device name */
#define MAX_CHARS 32
	char name[MAX_CHARS];
	int size;
	CHECK((size = ioctl(fd, EVIOCGNAME(MAX_CHARS), name)) < 0,
		exit, 1, "ioctl()");

	/* print device info */
	PRINT("%s [%i]: '%.*s'", argv[1], fd, size, name);

	/* print keymap */
	for (size_t i = 0; i < set.n; i++)
		PRINT("KEY%zu: %s [%hu]", i, keys[set.c[i]], set.c[i]);

	/* spawn the event listener */
	struct event_listener_arg arg = {&set, &child_exists_flag, fd};
	CHECK(pthread_create(&child_thread, NULL,
		(void *(*)(void *))event_listener, &arg),
		exit, 1, "pthread_create()");

	/* disable terminal echo */
	CHECK(tcgetattr(STDIN_FILENO, &attr), exit, 1, "tcgetattr()");
	attr.c_lflag &= ~(ECHO | ICANON);
	terminal_modified_flag = 1;
	CHECK(tcsetattr(STDIN_FILENO, TCSANOW, &attr), exit, 1, "tcsetattr()");

	PRINT("Main loop started.");

	struct timespec ts = {0, 10000000};

	while (loop)
	{
		unsigned int tmp;

		tmp = 0;
		for (size_t i = 0; i < set.n; i++)
			tmp |= (!!set.k[i] << i);

		code(tmp);

		/* slow down */
		(void) nanosleep(&ts, NULL);
	}

	PRINT("Main loop interrupted.");

	ret = 0;
	/* handle cleanup here */
exit: 
	/* close the input device if we opened it */
	if (fd != -1)
		(void) close(fd);
	
	/* clean up the event listener thread */
	if (child_exists_flag) {
		/* attempt to interrupt the thread */
		if (!pthread_kill(child_thread, SIGUSR1))
			PRINT("Waiting for event listener thread to exit.");
		void *tmp;
		(void) pthread_join(child_thread, &tmp);
		if (tmp) {
			ret = (size_t)tmp;
			PRINT("Event listener thread exited "
				"with nonzero status. (%i)", ret);
		}
	}

	/* restore terminal */
	if (terminal_modified_flag) {
		attr.c_lflag |= (ECHO | ICANON);
		(void) tcsetattr(STDIN_FILENO, TCSANOW, &attr);
	}

	PRINT("Main thread exit. (%i)", ret);

	return ret;
}

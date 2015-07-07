/*
 * sht.c - Testprogram for shared memory refclock
 * read/write shared memory segment; see usage
 */
#ifndef SYS_WINNT
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <unistd.h>
#define USLEEP(x) usleep(1000*x)
#else
#include <windows.h>
#define USLEEP(x) Sleep(x)
#endif
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <assert.h>
struct shmTime {
	int    mode; /* 0 - if valid set
		      *       use values,
		      *       clear valid
		      * 1 - if valid set
		      *       if count before and after read of values is equal,
		      *         use values
		      *       clear valid
		      */
	int    count;
	time_t ct_Sec;
	int    ct_USec;
	time_t rt_Sec;
	int    rt_USec;
	int    leap;
	int    precision;
	int    nsamples;
	int    valid;
    int    dummy[10];
};

struct shmTime *getShmTime (int unit)
{
#ifndef SYS_WINNT
	int shmid=shmget (0x4e545030+unit, sizeof (struct shmTime), IPC_CREAT|0666);
    if (shmid==-1) {
        shmdt((void *)(0x4e545030UL+unit));
	    shmid=shmget (0x4e545030+unit, sizeof (struct shmTime), IPC_CREAT|0666);
    }
	if (shmid==-1) {
		perror ("shmget");
        return 0;
	}
	else {
		struct shmTime *p=(struct shmTime *)shmat (shmid, 0, 0);
		if ((ssize_t)p==-1) {
			perror ("shmat");
			p=0;
		}
		return p;
	}
#else
    NTSTATUS r = STATUS_SUCCESS;
    OBJECT_ATTRIBUTES oa;
    LARGE_INTEGER SectionSize;
	SECURITY_DESCRIPTOR sd;
	SECURITY_ATTRIBUTES sa;
	HANDLE shmid;
    UNICODE_STRING SectionName;
    wchar_t buf[10];

    buf[0]=L'N'; buf[1]=L'T'; buf[2]=L'P';
    buf[4]=unit+0x30; unit[5]=0x00;
    RtlInitUnicodeString(&SectionName, buf);

    RtlCreateSecurityDescriptor(&sd, SECURITY_DESCRIPTOR_REVISION);
    RtlSetDaclSecurityDescriptor(&sd,1,0,0);
	sa.nLength=sizeof (SECURITY_ATTRIBUTES);
	sa.lpSecurityDescriptor=&sd;
	sa.bInheritHandle=0;

    memset(&oa, 0, sizeof(oa));
    oa.Length = sizeof(oa);
    oa.RootDirectory = NULL;
    oa.ObjectName = &SectionName;
    oa.Attributes = OBJ_CASE_INSENSITIVE;
    SectionSize.QuadPart = sizeof(struct shmTime);

    r = NtCreateSection(&shmid, 0x000F0007L, &oa, &SectionSize, PAGE_READWRITE, SEC_COMMIT, NULL);
    if (!NT_SUCCESS(r)) { /*error*/
        fprintf(stderr, "CreateFileMapping failed: (unit %u): 0x%lx", unit, r);
        return 0;
    }
	else {
        LARGE_INTEGER ofs;
        void *ViewBase = NULL;
        size_t ViewSize = sizeof(struct shmTime);
		struct shmTime *p = NULL;

        ofs.QuadPart = 0;
        r = NtMapViewOfSection(shmid, ((HANDLE)-1), &ViewBase, 0, 0, &ofs, &ViewSize, ViewShare, 0, PAGE_READWRITE);
		if (r != STATUS_SUCCESS) { /*error*/
			fprintf(stderr, "NtMapViewOfSection (unit %u): 0x%08lx", unit, r);
			return 0;
		} else {
            p = (struct shmTime *)ViewBase;
		    return p;
        }
		return p;
	}
	return 0;
#endif
}

void usage(void)
{
    const char *usage_str = "Usage: ntp_shmtool [-r|-w] [-cl] [-s N] [-p N]\n"
	"       -r read shared memory\n"
	"        -c clear valid-flag\n"
	"        -l loop (so, rcl will read (and optionally clear) in a loop\n"
	"       -w write shared memory with current time\n"
	"       -s N set nsamples to N\n"
	"       -p N set precision to -N\n";
    write(1, usage_str, strlen(usage_str));
}

int main (int argc, char *argv[])
{
	volatile struct shmTime *p;
    int c = 0, mode = 0, clear = 0, loop = 0;
    unsigned int nsamples = 0;
    unsigned char leap = 0;
    int precision = 0;
	while ((c = getopt(argc, argv, "clp:rs:L:wh")) != EOF)
	switch (c) {
	    case 's':
		    nsamples=atoi(optarg);
	        break;
	    case 'L':
		    leap=atoi(optarg);
	        break;
	    case 'p':
		    precision=-atoi(optarg);
	        break;
        case 'c':
            clear = 1;
            break;
        case 'l':
            loop = 1;
            break;
	    case 'r':
            mode = 0;
	        break;
	    case 'w':
            mode = 1;
	        break;
        case 'h':
        default:
            usage();
            return 0;
            break;
	}
    p=getShmTime(0x20);
    if (!p) {
        return -1;
    }
    if (nsamples) p->nsamples = nsamples;
    if (leap) p->leap = leap;
    if (precision) p->precision = precision;
    if (mode == 0) {
	    printf ("reader\n");
		do {
		    printf ("mode=%d, count=%d, clock=%ld.%d, rec=%ld.%d,\n",
				    p->mode,p->count,p->ct_Sec,p->ct_USec, p->rt_Sec,p->rt_USec);
			printf ("  leap=%d, precision=%d, nsamples=%d, valid=%d\n",
				    p->leap, p->precision, p->nsamples, p->valid);
			if (!p->valid)
				printf ("***\n");
			if (clear) {
				    p->valid=0;
				    printf ("cleared\n");
			}
			if (loop)
				USLEEP(950);
		} while (loop);
    } else {
		    printf ("writer\n");
		    p->mode=0;
		    if (!p->valid) {
			    p->ct_Sec=time(0)-20;
			    p->ct_USec=0;
			    p->rt_Sec=time(0)-1;
			    p->rt_USec=0;
			    printf ("%ld %ld\n",p->ct_Sec, p->rt_Sec);
			    p->valid=1;
		    }
		    else {
			    printf ("p->valid still set\n"); /* not an error! */
		    }
    }
    return 0;
}


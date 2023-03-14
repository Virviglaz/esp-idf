#ifndef __GENERIC_H__
#define __GENERIC_H__

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x)		(sizeof(x) / sizeof(x[0]))
#endif

#ifndef MIN
#define MIN(x, y)		(x) < (y) ? (x) : (y)
#endif

#ifndef MAX
#define MAX(x, y)		(x) > (y) ? (x) : (y)
#endif

#endif /* __GENERIC_H__ */

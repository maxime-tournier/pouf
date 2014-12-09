#ifndef POUF_TOOL_HERE_H
#define POUF_TOOL_HERE_H

#define TRACE2(f,l) "file: " f ",\tline: " #l
#define TRACE1(f,l) TRACE2(f,l)

#define HERE TRACE1(__FILE__,__LINE__)

#endif

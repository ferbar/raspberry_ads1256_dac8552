#include <stdio.h>
#include <boost/algorithm/string.hpp>
#include "utils.h"
#include <stdexcept>
#include <sstream>
#include <iostream>

// fürs backtrace:
#include <execinfo.h>
// fürs demangle:
#include <cxxabi.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#undef runtime_error
std::RuntimeExceptionWithBacktrace::RuntimeExceptionWithBacktrace(const std::string &what)
	: std::runtime_error(what)
{
	void *array[10];
	size_t size;

	/* --- print simple backtrace
	// get void*'s for all entries on the stack
	size = backtrace(array, 10);

	// print out all the frames to stderr das kamma mit addr2line noch anschaun dann:
	fprintf(stderr, "Error: %s size=%zd\n", what.c_str(), size);
	backtrace_symbols_fd(array, size, STDERR_FILENO);
	*/


	// Print Backtrace with demangle: - needs LDFLAGS -rdynamic
	// http://stackoverflow.com/questions/77005/how-to-generate-a-stacktrace-when-my-gcc-c-app-crashes

	char ** messages = backtrace_symbols(array, size);    

	// skip first stack frame (points here)
	for (size_t i = 1; i < size && messages != NULL; ++i) {
		char *mangled_name = 0, *offset_begin = 0, *offset_end = 0;
		// char *address_begin = 0, *address_end = 0;

		// find parantheses and +address offset surrounding mangled name
		// normalerweise sollte messages[i] so ausschaun: ./listing1(show_stackframe+0x2e) [0x80486de]
		// ohne -rdynamic fehlt der (show_stackframe+0x2e) - Teil!
		// addr2line verwendet die libdwarf, aber das dürften 300 zeilen benötigen
		// dladdr liefert nicht mehr Information als das backtrace()
		// LDFLAGS: -rdynamic hilft !!!
		for (char *p = messages[i]; *p; ++p) {
			if (*p == '(') {
				mangled_name = p; 
			} else if (*p == '+') {
				offset_begin = p;
			} else if (*p == ')') {
				offset_end = p;
				break;
			} /* else if (*p == '[') {
				address_begin = p;
			} else if (*p == ']') {
				address_end = p;
			} */
		}
		/*
		if(address_begin && address_end && (address_begin < address_end)) {
			long addr=strtol(address_begin+1,NULL,0);
			printf("addr=%#lx\n", addr);
			Dl_info info;
			int rc=dladdr((void *) addr, &info);
			printf("rc=%d, %s %p\n", rc, info.dli_fname, info.dli_saddr);
		}
		*/
		// printf("mangled_name %s=%p offset_begin %p offset_end %p \n", mangled_name, mangled_name, offset_begin, offset_end);
		// if the line could be processed, attempt to demangle the symbol
		if (mangled_name && offset_begin && offset_end && mangled_name < offset_begin) {
			*mangled_name++ = '\0';
			*offset_begin++ = '\0';
			*offset_end++ = '\0';

			int status;
			char * real_name = abi::__cxa_demangle(mangled_name, 0, 0, &status);

			// if demangling is successful, output the demangled function name
			if (status == 0) {    
				std::cerr << "[bt]: (" << i << ") " << messages[i] << " : " 
					<< real_name << "+" << offset_begin << offset_end 
					<< std::endl;

			}
			// otherwise, output the mangled function name
			else {
				std::cerr << "[bt]: (" << i << ") " << messages[i] << " : " 
					<< mangled_name << "+" << offset_begin << offset_end 
					<< std::endl;
			}
			free(real_name);
		}
		// otherwise, print the whole line
		else {
			std::cerr << "[bt]: (" << i << ") " << messages[i] << std::endl;
		}
	}
	std::cerr << std::endl;

	free(messages);
}

std::RuntimeExceptionWithBacktrace::~RuntimeExceptionWithBacktrace() throw ()
{

}


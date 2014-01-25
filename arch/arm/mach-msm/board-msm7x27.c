#if defined(CONFIG_MACH_CALLISTO)
#include "board-callisto.c"
#elif defined(CONFIG_MACH_EUROPA)
#include "board-europa.c"
#elif defined(CONFIG_MACH_COOPER_CTC)
#include "board-cooper-ctc.c"
#elif defined(CONFIG_MACH_COOPER)
#include "board-cooper.c"
#elif defined(CONFIG_MACH_BENI)
#include "board-beni.c"
#elif defined(CONFIG_MACH_TOTORO_CTC)
#include "board-totoro-ctc.c"
#elif defined(CONFIG_MACH_TASS_CTC)
#include "board-tass-ctc.c"
#elif defined(CONFIG_MACH_TASS)
#include "board-tass.c"
#elif defined(CONFIG_MACH_LUCAS)
#include "board-lucas.c"
#else
#include "board-msm7x27_org.c"
#endif

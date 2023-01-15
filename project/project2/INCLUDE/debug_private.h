



extern void UARTPrintf(const char *pcString, ...);

extern void UARTStdioInit(void);

#define debugPrint(X) UARTPrintf(X)



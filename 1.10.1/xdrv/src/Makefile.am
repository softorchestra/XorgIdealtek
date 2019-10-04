noinst_SCRIPTS = @WCM_XF86MODULES@

x86moduledir = @WCM_MODDIR@
x86module_DATA = @WCM_XF86MODULES@

# These identify which programs, libraries, and headers could
# potentially be built or installed depending on the results of
# the configuration.
EXTRA_SCRIPTS = xfdrvtouch_drv.o xfdrvtouch_drv.so

EXTRA_DIST = xfdrvtouch.c Ctrl_XInput.h	

if WCM_ENV_XORGSDK
DRIVER_INCLUDES =-I$(WCM_XORGSDK_DIR) $(X_CFLAGS)
endif

if WCM_ENV_XFREE86
DRIVER_INCLUDES =	-I$(WCM_XFREE86_DIR)/programs/Xserver/hw/xfree86/common \
	-I$(WCM_XFREE86_DIR)/programs/Xserver/hw/xfree86/loader \
	-I$(WCM_XFREE86_DIR)/programs/Xserver/hw/xfree86/os-support \
	-I$(WCM_XFREE86_DIR)/programs/Xserver/include \
	-I$(WCM_XFREE86_DIR)/programs/Xserver/mi \
	-I$(WCM_XFREE86_DIR)/exports/include/X11 \
	-I$(WCM_XFREE86_DIR)/include/extensions \
	-I$(WCM_XFREE86_DIR) \
	-I$(WCM_XFREE86_DIR)/exports/include
endif

NO_MERGE_CONSTANTS=@WCM_NO_MERGE_CONSTANTS@
NO_STACK_PROTECTOR=@WCM_NO_STACK_PROTECTOR@
WCM_CLEVEL=@WCM_CLEVEL@

if WCM_DLLOADER
DLLOADER=-fPIC
endif

XF86OBJS = xfdrvtouch.o

$(XF86OBJS):
	$(CC) $(CFLAGS) $(WCM_DEPFLAGS) $(DLLOADER) -pipe $(WCM_CLEVEL) \
		-pedantic -Wall -Wpointer-arith $(NO_MERGE_CONSTANTS) \
		$(NO_STACK_PROTECTOR) -I$(srcdir) $(DRIVER_INCLUDES) \
		$(WCM_XSERVER64) $(XSERVER_CFLAGS) \
		-o $@ -c $(srcdir)/$(subst .o,.c,$@)

if WCM_DLLOADER
xfdrvtouch_drv.so: .depend $(XF86OBJS)
	$(CC) -shared -nostdlib -o xfdrvtouch_drv.so $(XF86OBJS) -Bstatic -lgcc
else
xfdrvtouch_drv.o: .depend $(XF86OBJS)
	$(LD) -r $(XF86OBJS) -o xfdrvtouch_drv.o
endif

depend: .depend

.depend: Makefile
	@rm -f .depend
	$(CC) -MM $(CFLAGS) $(DEPFLAGS) $(DRIVER_INCLUDES) $(XF86OBJS:%.o=$(srcdir)/%.c) > .depend

-include .depend


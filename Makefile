

AUTOKD=autokd/autokd
WMT_LOADER=wmt_loader/wmt_loader
6620_LAUNCHER=6620_launcher/6620_launcher

all: $(AUTOKD) $(WMT_LOADER) $(6620_LAUNCHER)

.PHONY: clean

clean:
	make -C ./autokd clean
	make -C ./wmt_loader clean
	make -C ./6620_launcher clean

$(AUTOKD): 
	make -C ./autokd

$(WMT_LOADER):
	make -C ./wmt_loader

$(6620_LAUNCHER):
	make -C ./6620_launcher
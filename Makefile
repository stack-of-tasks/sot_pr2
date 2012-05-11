include $(shell rospack find mk)/cmake.mk

install: all
	make -C build install

.PHONY: install

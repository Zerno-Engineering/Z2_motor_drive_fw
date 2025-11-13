.PHONY: build

build:
	make fw_zerno_drive_clean; 
	make fw_zerno_drive V=1 -j $$(nproc); \
	echo ""; \
	printf "%s %02d:%02d:%02d\n" "Total Build Time:" "$$(( $$SECONDS / 3600 ))" "$$(( ( $$SECONDS / 60 ) % 60 ))" "$$(( $$SECONDS % 60 ))"; \
	echo ""

lint:
	uncrustify -q -c uncrustify.cfg --no-backup $$(find hwconf/Zerno/ -name "*.[ch]")

lint:
	$(TIDY) $(SRC) $(TIDY_OPT) -- $(CXXFLAGS) $(INC)

format:
	$(FORMAT) $(FORMAT_OPT) $(SRC)

.PHONY: lint format

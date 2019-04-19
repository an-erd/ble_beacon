1) Install 
	$ pip install nrfutil
	
2) generate private.key
	$ nrfutil.exe keys generate private.key
	Remark: gives some error messages but file created is fine

3) extract public_key.c
	$ nrfutil keys display --key pk --format code private.key --out_file public_key.c

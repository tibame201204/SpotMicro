import pymysql

link = pymysql.connect(
	host='localhost', 
	user='root', 
	passwd='', 
	db='my', 
	charset='utf8', 
	port=3306)

cur = link.cursor()
x = [
	  input("Title:"),
	  input("Description:"), 
	  input("Source:"), 
	  input("Create Time:"), 
	  input("URL:"), 
	  input("Photo URL"), 
	  input("Keyword:")
	  ]
cur.execute(
	"INSERT INTO `news`" + 
	"(`title`,`description`,`src`,`create_time`,`url`,`photo_url`,`keyword`) " + 
	"VALUES(%s,%s,%s,%s,%s,%s,%s)",
	x)
link.commit()
link.close()

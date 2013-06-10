import sqlite3


con = None

try:
    con = sqlite3.connect('aquaponic.db')
    
    cur = con.cursor()    
   # cur.execute('SELECT SQLITE_VERSION()')
    
   # data = cur.fetchone()
    
   # print "SQLite version: %s" % data                

    cur.execute("SELECT * FROM sensor_data")
    rows = cur.fetchall()

    for row in rows:
        print row

        
except lite.Error, e:
    
    print "Error %s:" % e.args[0]
    sys.exit(1)
    
finally:
    
    if con:
        con.close()

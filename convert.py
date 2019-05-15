end1 = '/Users/lucas/My Repo/Chinese Postman Problem/Instances/g_20_42.txt'
end2 = '/Users/lucas/My Repo/Chinese Postman Problem/Instances/g_20_42_2.txt'
with open(end1, "rb") as inf:
    with open(end2, "w") as fixed:
        for line in inf:
            #line = line.replace("\n", "")
            line = line.replace("\r", "\n")
            fixed.write(line)

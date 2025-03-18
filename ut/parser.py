import sys
import re
import json

def get_json(data):
    json_str = re.search(r'\{.*\}', data, re.DOTALL)
    # print(json_str)

    if not json_str:
        return None
    
    json_str = json_str.group()
    return json_str

def process_log_line(line):
    cleaned_line = re.sub(r'^\s*\d+\s+', '', line.strip())
    operation_match = re.search(r'\b(read|write)\b', cleaned_line, re.IGNORECASE)
    operation = operation_match.group(1).lower() if operation_match else 'unknown'

    processed_str = re.sub(r'^.*?(read|write)\s*', '', cleaned_line, flags=re.IGNORECASE).strip()
    
    get_json(processed_str)
    return [operation, get_json(processed_str)]

with open(sys.argv[1], 'r') as f:
    for line_num, line in enumerate(f, 1):
        line = line.replace('\\"', '"')

        result = process_log_line(line)
        # print(result)

        # print(result[1])

        if (not result[1]):
            continue

        # print(result[1])

        if 'write' == result[0]:
            # print(f'>>> {result[1]}')
            try:
                json_obj = json.loads(result[1])
                if json_obj['method'] != 'get_status':
                    print(json_obj['method'])
            except Exception as e:
                pass
        # else:
            # print(f'<<< {result[1]}')